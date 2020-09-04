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

import os
# 读取yaml格式配置文件
import yaml
# 解析kml文件
from pykml import parser

from Logger import Logger

# 角度计算'
import angle_utils


class Px4Controller:

    def __init__(self):
        rospy.init_node("test")

        current_path = os.path.abspath(
            os.path.dirname(__file__))

        config_path = current_path + '/config.yaml'

        # 读取配置文件
        with open(config_path, 'r') as f:
            config = yaml.load(f.read())
            self.set_config(config)

        log_path = current_path + "/log/all.log"
        # 保存15天内的日志文件
        self.log = Logger(log_path, level='debug', backCount=15)

        # kml的路径
        kml_path = current_path + "/" + self.kml_name

        print(kml_path)

        # 读取kml 文件
        self.kml_dict = {}
        self.read_kml_file(kml_path)

        self.dg = DiscreteGridUtils.DiscreteGridUtils()

        # 启动一个线程定时更新配置文件
        # Thread(target=self.read_config, args=(config_path,)).start()
        # imu状态
        self.imu = None
        # rc输入的状态
        self.rc_in = []
        # rc输入的状态
        self.rc_out = None

        self.yaw = 0.0    # 偏航角度
        self.parallel_yaw = 0.0  # 设置平行状态的yaw 值

        self.left_dist = 0.00   # 设置左侧距离

        # 点云分类
        self.pointsClassification = [[], [], [], []]

        # 所有点
        self.all_points = []

        # 均值滤波
        self.left_dist_queue = Queue()
        self.left_parallel_yaw_queue = Queue()

        # 当前gps 位置
        self.current_gps = None
        self.origin_gps = None
        self.init_gps = True
        
        # 当前位置距离航点的距离
        self.current_waypoint_dist = 0
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

        # 设置直线斜率
        self.line = None
        self.paths = None

        self.cloud_points = []

        self.num = 1
        # 延边模式下 航行状态 
        # 0 前往航点起始点
        # 1 沿航线航行
        # 2 前往过iao桥航点
        # 3 保持航向进行过桥
        # 4 过桥完成 前往过桥后与当前航向不超过70度的航点
        # 5 任务完成
        self.sailing_status = 0
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

        # 获取罗盘值
        self.GPS_sub = rospy.Subscriber(
            "/mavros/global_position/global", NavSatFix, self.GPS_callback)

        # 获取apm 状态
        self.state_sub = rospy.Subscriber(
            "/mavros/state", State, self.State_callback)
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
            "/voxel_grid/points", PointCloud2, self.point_cloud_callback)  # 订阅八叉树地图

        print("Px4 Controller Initialized!")

    def start(self, debug=False):
        rcmsg = OverrideRCIn()
        rcmsg.channels = [65535, 65535, 65535,
                          65535, 65535, 65535, 65535, 65535]
        # 是否启用画图
        if self.plot_debug:
            Thread(target=self.plot, args=()).start()
        first_cortron = True
        i = 0
        bridge_time = 0
        # 0 向桥首个航点驶入
        # 1 进入直行状态
        # 2 过桥完成,前往过桥后与当前航向不超过70度的航点
        bridge_status = 3
        while True:
            # 切入解锁
            # self.log.logger.info("arm_state:" + str(self.arm_state))
            # self.log.logger.info("manual_state:" + str(self.manual_state))
            if self.arm_state and self.manual_state and (rospy.is_shutdown() is False):
                rcmsg.channels[2] = 1700
                if self.current_gps != None and len(self.rc_in) > 2 and self.rc_in[5] > 1500:
                    if first_cortron == False:
                        self.log.logger.info("延边模式")
                        # 首次切入延边状态进入前往地一个航点
                        self.sailing_status = 0
                        first_cortron = True
                        i = 0
                        waypoint = self.current_gps
                    
                    # 判断当前位置与航点距离多少
                    self.current_waypoint_dist = self.lat_and_lon_get_distance(
                        self.current_gps, waypoint)

                    self.current_heading = self.lat_and_lon_get_angle1( self.current_gps, waypoint)


                    if bridge_status == 3:
                        min_bridge_waypoints = self.get_min_bridge()
                        # 获取距离当前位置与最近桥的距离以及桥本身航点
                        if  len(min_bridge_waypoints) > 0:
                            bridge_heading = self.lat_and_lon_get_angle1(  min_bridge_waypoints[0], min_bridge_waypoints[-1])
                            bridge_distance = self.lat_and_lon_get_distance(  min_bridge_waypoints[0], min_bridge_waypoints[-1])
                            self.log.logger.info(  "开始进行过桥,桥长" + str(bridge_distance) + "米 ,桥起始点: "+str(min_bridge_waypoints[0]))
                            # 满足过桥行动
                            start_bridge = True
                            bridge_time = 0
                            bridge_status = 0
                        
                        elif self.current_waypoint_dist < self.min_waypoint_distance:
                    
                            if i < len(self.kml_dict["left00"])  :
                                waypoint = self.kml_dict["left00"][i]
                                i += 1
                                self.log.logger.info( "前往第"+str(i)+"个航点:"+str(waypoint))
                            
                            elif i == len(self.kml_dict["left00"]) :
                                self.log.logger.info( "到达第"+str(i)+"个航点:"+str(waypoint))
                                self.log.logger.info("任务结束")
                                i+=1


                    # 若当前位置距离桥第一个点小于最小航点距离则进行
                    elif bridge_status == 0:
                        current_bridge_heading = self.lat_and_lon_get_angle1(  self.current_gps, min_bridge_waypoints[0])
                        current_bridge_distance = self.lat_and_lon_get_distance( self.current_gps, min_bridge_waypoints[0])
                        
                        self.set_yaw_pid_control(
                            current_bridge_heading, rcmsg)
                        if current_bridge_distance < self.min_waypoint_distance:
                            bridge_status = 1
                            start_time = time.time()

                    elif bridge_status == 1:
                      
                        self.set_yaw_pid_control(bridge_heading, rcmsg)
                        bridge_time = time.time()-start_time
                        if   bridge_time > bridge_distance / self.ground_speed:
                            self.log.logger.info(
                                "已通过桥,耗时"+str(bridge_time)+"秒,桥末尾点: "+str(min_bridge_waypoints[-1]))
                            # 确定通过桥梁
                            passing_bridge = False
                            bridge_status = 2
                        
                    elif bridge_status == 2 :
                        if i < len(self.kml_dict["left00"])-1 and angle_utils.angle_different_0_360(self.current_heading,self.yaw) >90:
                            waypoint = self.kml_dict["left00"][i]
                            self.log.logger.info( "跳过第"+str(i)+"个航点:"+str(waypoint))
                            i += 1
                        else:
                            self.log.logger.info( "前往第"+str(i)+"个航点:"+str(waypoint))
                            bridge_status = 3
                

                    # self.log.logger.info( "current_heading: " + str(self.current_heading))

                    # 判断点云是否符合沿岸要求，符合要求是时进行沿岸行走,不符合要求航向向下个航点行驶,或者进入调试模式下不进入延边模式
                    if len(self.cloud_points) > 5 and not self.yaw_PID_debug and bridge_status == 3:
                        self.get_dist_parallel_yaw()
                        if self.left_dist != None and self.left_parallel_yaw != None:
                            self.log.logger.info(
                                "left_dist: " + str(self.left_dist))
                            self.log.logger.info(
                                "left_parallel_yaw: "+str(self.left_parallel_yaw))

                            # 根据与岸的距离不断调整 yaw 是其不断逼近想要的距离
                            self.set_dist_pid_control(
                                self.except_dist, self.left_dist, self.left_parallel_yaw, rcmsg)
                    else:
                        # 向航点航向行驶
                        self.set_yaw_pid_control(self.current_heading, rcmsg)
                        time.sleep(0.1)
                # 如果切换到遥控模式
                elif len(self.rc_in) > 2 and self.rc_in[5] < 1500:

                    if first_cortron:
                        self.log.logger.info("遥控模式")
                        first_cortron = False
                        
                    rcmsg.channels[2] = 65535
                    rcmsg.channels[0] = self.rc_in[1]

                    time.sleep(0.1)
            # self.log.logger.info("rcmsg:" + str(rcmsg))
            self.rc_override_pub.publish(rcmsg)

    def get_min_bridge(self):

        # 当船距离桥小于最小值时认为需要进行过桥
        current_bridge_distance = self.min_bridge_distance
        min_bridge_waypoints = []
        bridge_heading = None
        # 判断桥的的位置距离有多远,返回船最近距离桥的最近点且与下个航点的的航向,与当前位置与桥的航向超过180度时,忽略
        for key in self.kml_dict:
            if "bridge" in key:
                # self.log.logger.info("bridge")
                # 获取桥的中线位置,当得到船距离桥只有15m 时进入过桥模式固定yaw值

                start_bridge_distance = self.lat_and_lon_get_distance(
                    self.current_gps, self.kml_dict[key][0])
                end_bridge_distance = self.lat_and_lon_get_distance(
                    self.current_gps, self.kml_dict[key][-1])

                if start_bridge_distance < current_bridge_distance:
                    current_bridge_distance = start_bridge_distance
                    min_bridge_waypoints = self.kml_dict[key]
                    bridge_heading = self.lat_and_lon_get_angle1(
                        self.kml_dict[key][0], self.kml_dict[key][-1])

                if end_bridge_distance < current_bridge_distance:
                    current_bridge_distance = end_bridge_distance
                    min_bridge_waypoints = self.kml_dict[key][::-1]
                    bridge_heading = self.lat_and_lon_get_angle1(
                        self.kml_dict[key][-1], self.kml_dict[key][0])

        if bridge_heading == None or angle_utils.angle_min_different_0_360(bridge_heading, self.yaw) > 70  :
            min_bridge_waypoints = []

        # 返回当前位置与桥的航点位置
        return min_bridge_waypoints

    def lat_and_lon_get_angle1(self, Waypoint_start, Waypoint_end):
        '''
        计算两个经纬度之间的方位角
        Waypoint_start : 起始点(经度，纬度)
        Waypoint_end ： 指向末尾点(经度，纬度)
        '''

        y = math.sin(Waypoint_end[0]-Waypoint_start[0]
                     ) * math.cos(Waypoint_end[1])
        x = math.cos(Waypoint_start[1])*math.sin(Waypoint_end[1]) - math.sin(Waypoint_start[1]) * \
            math.cos(Waypoint_end[1]) * \
            math.cos(Waypoint_end[0]-Waypoint_start[0])
        brng = math.atan2(y, x)*180/math.pi

        if brng < 0:
            brng = brng + 360
        return brng

    def lat_and_lon_get_distance(self, Waypoint_start, Waypoint_end):
        '''
        计算两个经纬度之间的距离
        Waypoint_start : 起始点(经度，纬度)
        Waypoint_end ： 指向末尾点(经度，纬度)
        '''
        R = 6378137  # 地球半径
        wps_y = Waypoint_start[1] * math.pi / 180.0
        wpe_y = Waypoint_end[1] * math.pi / 180.0
        a = wps_y - wpe_y
        b = (Waypoint_start[0] - Waypoint_end[0]) * math.pi / 180.0

        sa2 = math.sin(a / 2.0)
        sb2 = math.sin(b / 2.0)
        d = 2 * R * math.asin(math.sqrt(sa2 ** 2 +
                                        math.cos(wps_y) * math.cos(wpe_y) * sb2 ** 2))
        return d

    def read_kml_file(self, kml_path):
        '''
        读取kml文件
        '''
        with open(kml_path) as f:
            doc = parser.parse(f).getroot()

        Placemarks = doc.Document.Folder.Placemark
        for Placemark in Placemarks:
            gps_queue = []
            All_GPS = Placemark.LineString.coordinates.text.replace(
                '\n', '').replace('\t', '').replace('0 ', '')
            gps_list = All_GPS.split(",")
            for i in range(0, int((len(gps_list)-1)/2)):
                gps_queue.append(
                    (float(gps_list[2*i]), float(gps_list[2*i+1])))

            self.kml_dict[Placemark.name.text] = gps_queue

    def read_config(self, config_path):
        '''
        循环判断配置文件是否更新，更新之后重新读取文件，更新配置文件
        '''
        start_time = os.stat(config_path).st_mtime
        while True:
            time.sleep(1)
            if os.stat(config_path).st_mtime != start_time:
                start_time = os.stat(config_path).st_mtime
                with open(config_path, 'r') as f:
                    config = yaml.load(f.read())
                    self.set_config(config)

    def set_config(self, config):
        '''
        设置参数
        '''
        self.except_dist = config["except_dist"]  # 期望与岸的距离为 0.7
        # 设置转向 pid设置，调试时应先调整转向pid 保证能很好地转到某个yaw值
        self.yaw_PID_debug = config["yaw_PID_debug"]  # 是否调试yawPID
        # 调试Yaw PID时应固定一个yaw看是否能 很好地转到需要方向
        self.yaw_PID_yaw = config["yaw_PID_yaw"]
        self.course_range = config["course_range"]  # 设置航向范围，在原始航点最大偏转30度
        yaw_P = config["yaw_P"]
        yaw_I = config["yaw_I"]
        yaw_D = config["yaw_D"]
        # 设置yaw pid设置
        self.yaw_pid = PID.PID(P=yaw_P, I=yaw_I, D=yaw_D)

        # 设置距离 pid设置，
        dist_P = config["dist_P"]
        dist_I = config["dist_I"]
        dist_D = config["dist_D"]
        # 设置dist_pid pid设置
        self.dist_pid = PID.PID(P=dist_P, I=dist_I, D=dist_D)

        # 多少值参与均值滤波 越大表示反应越缓慢
        self.filter_N = config["filter_N"]
        # 是否画出岸与船的关系
        self.plot_debug = config["plot_debug"]
        # 是否写出日志
        self.log_debug = config["log_debug"]
        # 是否打印日志
        self.print_log_debug = config["print_log_debug"]

        # kml 文件名
        self.kml_name = config["kml_name"]

        # 最小航点距离，当船距离航点距离最小距离时，认为已经到达航点,单位m
        self.min_waypoint_distance = config["min_waypoint_distance"]

        # 执行过桥最近距离,小于此距离执行过桥动作
        self.min_bridge_distance = config["min_bridge_distance"]

        # 地面速度
        self.ground_speed = config["ground_speed"]

    def avg_filter(self, queue, value, filter_N):
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
            return round(((sum(list_sort)-list_sort[0]-list_sort[-1])/(filter_N-2)), 6)
        else:
            return None

    def plot(self):
        while self.plot_debug:

            if self.line != None:

                slope = -1/math.tan(self.line.theta)
                paths = self.paths
                intercept = self.line.intercept

                plt.cla()
                plt.axis([-15, 0, -3, 7])
                # 画出直线
                a = [-10, 0]
                b = [slope*a[0]+intercept, slope*a[1]+intercept]

                plt.plot(a, b, "g-", linewidth=2.0)

                paths.append((0, 0))
                c = []
                d = []
                for item in paths:
                    c.append(item[0])
                    d.append(item[1])
                # plt.scatter(c, d, alpha=0.5, marker=",")
                plt.scatter(c, d, alpha=0.5, marker=",")

                plt.pause(1)

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

        # print ("except_yaw", except_yaw)
        # 期望值yaw 在执行任务时最大值不得与当前 位置到航点位置的方位角相差30度
        except_yaw = self.except_yaw_range(except_yaw)
        # 根据期望yaw 值不断调整车身与岸距离
        self.set_yaw_pid_control(except_yaw, rcmsg)

    def except_yaw_range(self, except_yaw):
        '''
        根据期望角度范围设置期望角度
        '''
        current_heading_up = self.current_heading + self.course_range
        current_heading_down = self.current_heading - self.course_range

        if (current_heading_up > 360 or current_heading_down < 0) and ((current_heading_up) % 360 < except_yaw < (current_heading_down) % 360):
            if except_yaw - (current_heading_up) % 360 < (current_heading_down) % 360 - except_yaw:
                except_yaw = (current_heading_up) % 360
            else:
                except_yaw = (current_heading_down) % 360
        elif 0 < except_yaw < current_heading_down:
            if except_yaw + 360 - (current_heading_up) < current_heading_down - except_yaw:
                except_yaw = current_heading_up
            else:
                except_yaw = current_heading_down
        elif current_heading_up < except_yaw < 360:
            if except_yaw - (current_heading_up) < current_heading_down + 360 - except_yaw:
                except_yaw = (current_heading_up) % 360
            else:
                except_yaw = current_heading_down
        return except_yaw

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
        # self.yaw = msg.data
        pass

    def point_cloud_callback(self, msg):
        '''
        获取点云信息 
        '''

        self.cloud_points = list(pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True))

    def get_dist_parallel_yaw(self):
        # 第一象限数
        quadrant_1 = set()
        quadrant_2 = set()
        quadrant_3 = set()
        quadrant_4 = set()
        all_points = []
        center_point = (0, 0)
        for point in self.cloud_points:
            h_top = 0.5
            h_down = 0.01
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
        self.pointsClassification[0] = sorted(
            quadrant_1, key=lambda item: item.k)
        self.pointsClassification[1] = sorted(
            quadrant_2, key=lambda item: item.k)
        self.pointsClassification[2] = sorted(
            quadrant_3, key=lambda item: item.k)
        self.pointsClassification[3] = sorted(
            quadrant_4, key=lambda item: item.k)

        self.all_points = all_points
        cen = CenterPointEdge.CenterPointEdge(
            self.pointsClassification, center_point, 2)
        self.paths = cen.contour()
        if len(self.paths) > 5:
            self.line = LineSegment.segment_Trig(
                self.paths, center_point=center_point)
            # 当线段拟合程度不够时使用双折线拟合
            if self.line.err > 0.01:
                lines, good_i = LineSegment.segment_Trig_2(
                    self.paths, center_point=center_point)
                # 双折线中选择k 值为负值且拟合程度大于0.75的线
                # 次选拟合程度最大的
                if good_i > len(self.paths)-good_i and lines[0] != None:
                    self.line = lines[0]
                elif lines[1] != None:
                    self.line = lines[1]
        self.num += 1
        self.paths.append((0, self.num))
        # 均值滤波
        self.left_dist = self.avg_filter(
            self.left_dist_queue, self.line.dist, self.filter_N)
        # 岸线的平行的角度
        deflection = self.line.theta*180/math.pi
        left_parallel_yaw = (self.yaw - deflection) % 360
        # 均值滤波
        self.left_parallel_yaw = self.avg_filter(
            self.left_parallel_yaw_queue, left_parallel_yaw, self.filter_N)

    def points_dist(self, point1, point2):
        '''
        计算两点之间的距离
        point1 : (x,y)360
        '''
        return math.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)

# imu 回调函数
    def imu_callback(self, msg):
        self.imu = msg
        heading = self.q2yaw(self.imu.orientation)
        self.yaw = (-heading / math.pi * 180.0 + 90) % 360

    # rc输入的回调函数

    def rc_in_callback(self, msg):
        # 将rc输入的通道值赋给rc_in
        self.rc_in = msg.channels
        # self.log.logger.info(self.rc_in)

    # rc输出的回调函数
    def rc_out_callback(self, msg):
        # 将rc输入的通道值赋给rc_in
        self.rc_out = msg.channels
        

    # 速度回调函数

    def vfr_hub_callback(self, msg):
        # self.ground_speed = msg.groundspeed
        pass

    def GPS_callback(self, msg):
        '''
        获取GPS 信息
        msg: 消息格式

        header: 
            seq: 110
            stamp: 
                secs: 1598234193
                nsecs: 418708509
            frame_id: "base_link" 
        status: gps 状态
            status: -1
            service: 1
        latitude: 0.0  纬度
        longitude: 0.0  经度
        altitude: 16.433    高度
        position_covariance: [18446744065119.617, 0.0, 0.0, 0.0, 18446744065119.617, 0.0, 0.0, 0.0, 14184584949071.611] 位置协方差
        position_covariance_type: 2
        '''
        self.current_gps = (msg.longitude, msg.latitude)
        self.GPS_status = msg.status.status

    '''
    return yaw from current IMU
    '''

    def q2yaw(self, q):
        q_ = Quaternion(q.w, q.x, q.y, q.z)
        rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad

    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def State_callback(self, msg):
        self.state = msg
        self.arm_state = msg.armed
        self.manual_state = (msg.mode == "MANUAL")

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
    con.start(debug=True)
