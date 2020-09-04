# -*- coding: utf-8 -*-
# 引入ros的Python包
import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget, OverrideRCIn, RCIn, RCOut, VFR_HUD
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2


from std_msgs.msg import Float32, Float64, String

# 引入点云信息
import sensor_msgs.point_cloud2 as pc2

import time
from pyquaternion import Quaternion
import math
from threading import Thread

# 计算散点斜率
import numpy as np
import pandas as pd
# 引入队列
from Queue import Queue
# 新建传感器队列
# 引入PID 设置
import PID
# 引入tf 转换
import tf

# 引入离散化
import DiscreteGridUtils

# 获取围绕中心点的边线
import CenterPointEdge

import LineSegment
# 线程
import threading

import matplotlib.pyplot as plt
import matplotlib
import time

import scipy.stats as st


class Px4Controller:

    def __init__(self):
        rospy.init_node("test")
        self.dg = DiscreteGridUtils.DiscreteGridUtils()

        # imu状态
        self.imu = None
        # rc输入的状态
        self.rc_in = None
        # rc输入的状态
        self.rc_out = None

        self.yaw = 0.0    # 偏航角度
        self.parallel_yaw = 0.0  # 设置平行状态的yaw 值

        # 地速
        self.ground_speed = 0.3

        self.except_dist = 1  # 期望与岸的距离为 0.7
        self.left_K = 0.00  # 车身左侧与岸线平行的斜率值
        self.left_K = 0.00  # 车身右侧与岸线平行的斜率值
        # self.sensor_dist = 0.5 #两侧传感器距离

        self.left_dist = 0.00   # 设置左侧距离
        self.left_dist = 0.00  # 设置右侧距离
        # 设置yaw pid设置
        self.yaw_pid = PID.PID(P=15.0, I=0.00, D=0.00)

        # 设置dist_pid pid设置
        self.dist_pid = PID.PID(P=30.0, I=0.0, D=0.0)

        # 点云分类
        self.pointsClassification = [0, 0, 0, 0]

        # 所有点
        self.all_points = []

        # s十个参与均值滤波的值
        self.filter_N =10
        # 均值滤波
        self.left_dist_queue = Queue()
        self.left_parallel_yaw_queue = Queue()


        self.gps = None
        self.local_pose = None
        self.current_state = None
        self.current_heading = 1
        self.takeoff_height = 3
        self.local_enu_position = None
        self.cur_target_pose = None
        self.global_target = None
        self.received_new_task = False
        self.arm_state = False
        # 手动模式切换状态
        self.manual_state = False
        self.received_imu = False
        self.frame = "BODY"
        self.state = None
        #
        self.left_parallel_yaw = None
        # 点云数据
        self.point_cloud = None
        # 设置点云锁
        self.point_set_mutex = threading.Lock()
        '''
        ros subscribers
        '''
        # self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.imu_sub = rospy.Subscriber(
            "/mavros/imu/data", Imu, self.imu_callback)
        self.rc_in_sub = rospy.Subscriber(
            "/mavros/rc/in", RCIn, self.rc_in_callback)

        self.rc_out_sub = rospy.Subscriber(
            "/mavros/rc/out", RCOut, self.rc_out_callback)

        self.vfr_hud_sub = rospy.Subscriber(
            "/mavros/vfr_hud", VFR_HUD, self.vfr_hub_callback)
        # 获取罗盘值
        self.compass_hdg_sub = rospy.Subscriber(
            "/mavros/global_position/compass_hdg", Float64, self.compass_hdg_callback)
        '''
        ros publishers
        '''
        # rc重载发布
        self.rc_override_pub = rospy.Publisher(
            'mavros/rc/override', OverrideRCIn, queue_size=10)
        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy(
            '/mavros/set_mode', SetMode)

        self.octomap_cells_vis = rospy.Subscriber(
            "/voxel_grid/output", PointCloud2, self.point_cloud_callback)  # 订阅八叉树地图

        print("Px4 Controller Initialized!")

    def start(self):

        arefa = 20
        center_point = (0, 0)

        for i in range(10):
            #  解除锁定
            self.arm_state = self.arm()
            self.manual_state = self.manual()
            time.sleep(0.2)
        r = rospy.Rate(10)  # 设置数据发送频率为10hz
        rcmsg = OverrideRCIn()
        rcmsg.channels = [65535, 65535, 65535,
                          65535, 65535, 65535, 65535, 65535]
        # 定义油门
        # rcmsg.channels[2] = 1500

        # 启动获取距离
        # Thread(target=self.get_dist, args=()).start()
        # 设置pid
        # Thread(target=self.set_pid,args=()).start()
        first_cortron = True
        while self.arm_state and self.manual_state and (rospy.is_shutdown() is False):

            # print("1")
            #
            if len(self.pointsClassification[3]) >= 0 and self.rc_in[5] > 1500:
                first_cortron = True
                # 清除原有图像
                start_time = time.time()
                cen = CenterPointEdge.CenterPointEdge(
                    self.pointsClassification, center_point, 1)
                paths = cen.contour()

                # print(paths)
                # 计算单折线斜率
                slope, intercept, r_value, p_value, std_err = st.linregress(paths)

                # 当线段拟合程度不够时使用双折线拟合
                if r_value**2 <0.75:
                    lines = LineSegment.segment2(
                        paths, center_point=center_point, debug=False, debug_data=paths)
                    # 双折线中选择k 值为负值且拟合程度大于0.75的线
                    # 次选拟合程度最大的
                    # print (lines    )
                    if (lines[0].slope > 0 and lines[0].r_value >0.75):
                        slope = lines[0].slope
                        intercept = lines[0].intercept
                    elif lines[0].r_value <0.75 and lines[1].r_value == 1.0:
                        slope = float("inf")
                        intercept = 0
                    else:
                        slope = lines[1].slope
                        intercept = lines[1].intercept

                self.plot(paths,slope,intercept)

                # 线性拟合，可以返回斜率，截距，r 值，p 值，标准误差
                # slope*x - y + intercept = 0
                x = []
                for path in paths:
                    x.append(path[0])

                if slope == float("inf"):
                    deflection = 0
                    # 平均x 值当做距离
                    left_dist = abs( list (np.average(paths,axis=0))[0])
                else:
                      
                    deflection = math.atan(slope)*180.0/math.pi
                    if deflection >0 :
                        deflection -=90 
                    else:
                        deflection +=90 
                    # 船到岸边的距离
                    left_dist = abs(
                        center_point[0]*slope-center_point[0]+intercept)/math.sqrt(slope**2+1)
                # 均值滤波
                self.left_dist =  self.avg_filter(self.left_dist_queue,left_dist,self.filter_N)
                # print ("defle",deflection)
                # 岸线的平行的角度
                left_parallel_yaw = (self.yaw - deflection) % 360
                # 均值滤波
                self.left_parallel_yaw =  self.avg_filter(self.left_parallel_yaw_queue,left_parallel_yaw,self.filter_N)

                # print ("left_parallel_yaw: ",self.left_parallel_yaw  )
                end_time = time.time()      
                # print ("time1: ", end_time-start_time)
                if self.left_dist !=None and self.left_parallel_yaw != None :
                    # 根据与岸的距离不断调整 yaw 是其不断逼近想要的距离
                    self.set_dist_pid_control(
                        self.except_dist, self.left_dist, self.left_parallel_yaw, rcmsg)
                    # 调试直行程序pid
                    # self.set_yaw_pid_control(311,rcmsg)
                    # print ("rcmsg: ",rcmsg)
                    # self.print_log()  # 打印日志
                    self.rc_override_pub.publish(rcmsg)
                # time.sleep(0.1)
            else:
                # rcmsg.channels = [65535, 65535, 65535,
                #                 65535, 65535, 65535, 65535, 65535]
                if first_cortron:
                    rcmsg.channels[0] = 1500
                    first_cortron = False
                else:
                    rcmsg.channels[0] = 65535

                # print ("rcmsg: ",rcmsg)
                # print ("self.rc_in", self.rc_in)
                # self.print_log()  # 打印日志
                self.rc_override_pub.publish(rcmsg)
                time.sleep(0.1)

    def avg_filter(self,queue,value,filter_N ):
        '''
        均值滤波函数
        queue : 储存滤波队列
        value : 滤波的值
        filter_N : n个平均值参与滤波
        '''
        queue.put(value)
        # 均值滤波
        if queue.qsize() == filter_N:
            list_sort = list(queue.queue)
            # 弹出队列中数据
            queue.get()
            list_sort.sort()
            # 去除一个最大值一个最小值 求平均获得最终结果
            return round(((sum(list_sort)-list_sort[0]-list_sort[-1])/(filter_N-2)),6)
        else :
            return None

    def print_log(self,):
        '''
        打印日志
        '''
        print ("self.yaw", self.yaw)
        print ("self.left_parallel_yaw", self.left_parallel_yaw)
        print ("except_dist: ", self.except_dist)
        print ("self.left_dist", self.left_dist)
        print ("ground", self.ground_speed)
        print ("self.rc_in", self.rc_in)
        print ("self.rc_out", self.rc_out)
        print ("=================================")
    
    def plot (self,paths,slope,intercept):
        plt.cla()

                # 画出直线
        a = [paths[0][0],paths[-1][0]]
        b = [slope*paths[0][0]+intercept,slope*paths[-1][0]+intercept]
        
        plt.plot(a, b, "g-", linewidth=2.0)

        paths.append((0,0))
        # paths.append((0,3))
        # paths.append((-2,-1))
        c = []
        d = []
        for item in paths:
            c.append(item[0])
            d.append(item[1])
        # plt.scatter(c, d, alpha=0.5, marker=",")
        plt.scatter(c, d, alpha=0.5, marker=",")


    
        # plt.plot(a, b, "r-", linewidth=2.0)
        #     # 显示图形
        # plt.show()
          # # 暂停
        # print("lines",lines)
        
        plt.pause(0.1)
    def set_dist_pid_control(self, except_dist, now_dist, parallel_yaw, rcmsg):
        '''
        根据期望离岸距离 不断调整yaw 到达期望距离
        except_dist 期望到达的距离,
        now_dist 当前距离值,
        parallel_yaw  车身与岸平行时的yaw 值
        rcmsg   设置通道值 
        '''
        self.dist_pid.SetPoint = except_dist
        self.dist_pid.update(float(now_dist))
        self.dist_pid.sample_time = 0.1  # 采样间隔
        except_yaw = int(parallel_yaw + self.dist_pid.output)  # 得到的期望yaw 值
        # 期望yaw 应在0-360 之间
        if except_yaw > 360:
            except_yaw = except_yaw % 360
        elif except_yaw < 0:
            except_yaw = except_yaw % 360 + 360

        print ("except_yaw", except_yaw)

        # 根据期望yaw 值不断调整车身与岸距离
        self.set_yaw_pid_control(except_yaw, rcmsg)

    def set_yaw_pid_control(self, except_yaw, rcmsg):
        '''
        根据期望yaw 值调整pwm 到达期望yaw 值
        except_yaw 期望到达的yaw值,
        rcmsg   设置通道值 
        '''
        self.yaw_pid.SetPoint = except_yaw
        self.yaw_pid.yaw_update(float(self.yaw))  # 针对0 -360 yaw 值针对 设置pid
        self.yaw_pid.sample_time = 0.1  # 采样间隔
        rcmsg.channels[0] = int(1500 + self.yaw_pid.output)
        # 设定pwm 值最大为2000 最小为1000
        if rcmsg.channels[0] > 2000:
            rcmsg.channels[0] = 2000
        elif rcmsg.channels[0] < 1000:
            rcmsg.channels[0] = 1000

    def set_pid(self):
        while True:
            P1 = float(raw_input("请输入P值："))
            I1 = float(raw_input("请输入I值："))
            D1 = float(raw_input("请输入D值："))
            self.yaw_pid = PID.PID(P=P1, I=I1, D=D1)
            print ("你已更新PID值为", P1, I1, D1)


# 设置为手动模式


    def manual(self):
        if self.flightModeService(custom_mode='MANUAL'):
            return True
        else:
            print("Vechile MANUAL failed")
            return False

    '''
    罗盘数据回调函数

    msg 罗盘获取到的yaw 值
    '''

    def compass_hdg_callback(self, msg):
        self.yaw = msg.data
        # if -0.01<self.left_K < 0.01 : #根据斜率判断平行岸的值
        # self.left_parallel_yaw = 110

        # if -0.01 < self.left_K < 0.01:  # 根据斜率判断平行岸的值
        # self.left_parallel_yaw = self.yaw

    def point_cloud_callback(self, msg):
        '''
        获取点云信息 
        '''
        # 第一象限数
        quadrant_1 = set()
        quadrant_2 = set()
        quadrant_3 = set()
        quadrant_4 = set()
        all_points = []
        center_point = (0, 0)

        # print("len   : ",len(list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))))
        start_time = time.time()
        for point in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            h_top = 1
            h_down = 0.4
            # print (type(point))
            # 获取岸线高层数据
            if point[2] < h_top and point[2] > h_down:
                # 将点栅格化
                grille_point = self.dg.continuous_to_discrete(
                    (point[0], point[1], point[2]))

                if grille_point not in all_points:
                    all_points.append(grille_point)

                    dist = self.points_dist(grille_point, center_point)

                    if (grille_point[0] - center_point[0]) != 0:

                        k = (grille_point[1] - center_point[1]) / \
                            (grille_point[0] - center_point[0])
                    else:
                        k = float("inf")
                    # print("k : ", k)
                    # 判断点云在哪个象限内
                    if grille_point[1] > 0:
                        if grille_point[0] > 0:
                            # 第一象限
                            quadrant_1.add(
                                PointRelationship(dist, 1, k, (grille_point[0], grille_point[1])))
                        else:
                            # 第二象限
                            quadrant_2.add(
                                PointRelationship(dist, 2, k, (grille_point[0], grille_point[1])))
                    else:
                        if grille_point[0] > 0:
                            # 第四象限
                            quadrant_4.add(
                                PointRelationship(dist, 4, k, (grille_point[0], grille_point[1])))
                        else:
                            # 第三象限
                            quadrant_3.add(
                                PointRelationship(dist, 3, k, (grille_point[0], grille_point[1])))
        # end_time = time.time()
        # print("all_points :  ",len(all_points))
        # 可能会出现线程冲突，需要加锁保证同时改变
        acquired = self.point_set_mutex.acquire(
            True)  # blocking.  尝试获得锁定。使线程进入同步阻塞状态
        if acquired:
            self.pointsClassification[0] = sorted(
                quadrant_1, key=lambda item: item.k)
            self.pointsClassification[1] = sorted(
                quadrant_2, key=lambda item: item.k)
            self.pointsClassification[2] = sorted(
                quadrant_3, key=lambda item: item.k)
            self.pointsClassification[3] = sorted(
                quadrant_4, key=lambda item: item.k)
            self.point_set_mutex.release()  # 释放锁
            # end_time = time.time()
            # print("time :  ",end_time-start_time)
            self.all_points = all_points
            return
        else:
            print ('Lock not acquired!')

    def points_dist(self, point1, point2):
        '''
        计算两点之间的距离
        point1 : (x,y)
        '''
        return math.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)

# imu 回调函数
    def imu_callback(self, msg):
        global global_imu, current_heading
        self.imu = msg
        # print ("imu",msg)
        self.current_heading = self.q2yaw(self.imu.orientation)
        # self.yaw = self.current_heading / math.pi * 180.0
        # print (" yaw ",self.yaw)
        self.received_imu = True

    # rc输入的回调函数
    def rc_in_callback(self, msg):
        # 将rc输入的通道值赋给rc_in
        self.rc_in = msg.channels

    # rc输出的回调函数
    def rc_out_callback(self, msg):
        # 将rc输入的通道值赋给rc_in
        self.rc_out = msg.channels

    # 速度回调函数

    def vfr_hub_callback(self, msg):
        # self.ground_speed = msg.groundspeed
        pass

    '''
    return yaw from current IMU
    '''

    def q2yaw(self, q):
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad

    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False


class PointRelationship:
    def __init__(self, distent, quadrant, k, point):
        '''
        点与环绕的中心点之间的关系
        distent：两点之间的距离
        quadrant ：点在第几象限
        k：点相对环绕中心点的k 值
        point: 点的具体坐标 （x,y）
        '''
        self.distent = distent
        self.quadrant = quadrant
        self.k = k
        self.point = point


if __name__ == '__main__':

    con = Px4Controller()
    con.start()
