#encoding=utf-8

'''
        project overview:

	Subscribe:
		1.slam pose(global/local pose) *
		2.octomap_server/global map
		3.local pointcloud/local octomap
		4.target input(semantic target/visual pose target/gps target)
	Publish:
		1.Mavros(amo) Command
		2.Navigator status

	Algorithms:
		1.D*
		2.state transfer
		3.position->position PID controller
		4.global/semantic/visual target to local pose
'''

import math
import thread
import threading
import time
from enum import Enum


import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# other useful utilities
from pyquaternion import Quaternion
# import pyquaternion
# for ros
import rospy
import sensor_msgs.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, Twist
# for mavros
from mavros_msgs.msg import (VFR_HUD, GlobalPositionTarget, OverrideRCIn,
                             PositionTarget, RCIn, State)
from mavros_msgs.srv import CommandBool, SetMode
# for octomap
from octomap_msgs.msg import Octomap, OctomapWithPose, octomap_msgs
from sensor_msgs.msg import Imu, NavSatFix, PointCloud, PointCloud2
from std_msgs.msg import Float32, Float64, String
from visualization_msgs.msg import Marker, MarkerArray

from nav_msgs.msg import Odometry


import astar.astar
import astar.driver
import DiscreteGridUtils
# import PID
# import random
# import time
from commander import Commander
from path_optimization.path_pruning import PathPruning
#from Pos2PosController import Pos2PosController as Controller  # TODO:re-implement this.
from SimController import Controller as Controller

#from queue import Queue

import traceback  



# define system status
class status(Enum):
    INITIALIZED = 1

    LOOKING_FOR_PATH = 2
    LOOKING_FOR_PATH_SUCCEED = 3
    LOOKING_FOR_PATH_FAILED = 4

    GOING_TO_TARGET = 5
    GOING_TO_VISION_TARGET = 6
	

def dist(pos1,pos2):
    if not pos1 or not pos2:
        return False, 0
    else:
        return True, reduce(lambda x,y:x+y,map(lambda i:(pos1[i]-pos2[i])**2,[0,1,2]))

class Navigator:
    
    def __init__(self,config_file_path = None):
        if config_file_path:
            pass


        rospy.init_node("gi_navigator_node")

        self.ground_h = 10 # 设置地面高度
        self.alpha = 0.2 # 设置alpha 的值控制 获取边界
        self.dist = 9 # 距离最近点的距离计算下一点

        self.error_yaw = 0 # 设置yaw 的偏差
        self.current_yaw = 0 # 当前的yaw 值

        self.except_dist = 5 # 期望与岸的距离为 0.7

        self.bank_status = True #记录岸的状态 True 左岸，False右岸 
        self.bank_dist = 0 #船距离岸的距离
        self.bank_parallel_yaw = 0 # 岸的yaw 值
        # # 设置yaw pid设置
        # self.yaw_pid = PID.PID(P=6.0,I=0.00 ,D=0.00)

        # # 设置dist_pid pid设置
        # self.dist_pid = PID.PID(P=20.0,I=0.0 ,D=0.0)

        self.arm_state = False
        # 手动模式切换状态
        self.manual_state = False
        # 获取设置点的位置
        self.set_point_pos =None

        self.dg = DiscreteGridUtils.DiscreteGridUtils(grid_size=0.2)#0.2
        self.rate = rospy.Rate(50)
        self.driver = astar.driver.Driver()
        self.controller = Controller()
        # 设置为手动模式
        self.mavros_state = "MANUAL"
        self.set_status(status.INITIALIZED) 

        self.current_pos =None

        self.cur_command_id = 0
        self.prev_command_id = 0
        self.cur_target_position=None

        self.task_id = -1
        self.obstacle_set_mutex = threading.Lock()  # mutex.acquire(timeout);mutex.release()
        self.nav_command_mutex = threading.Lock()  # for nav command in dstar and ros high level command.
        self.local_pose = None
        t1 = threading.Thread(target=self.ros_thread)
        t1.start()

        self.navigator_status_pub = rospy.Publisher('/gi/navigator_status', String, queue_size=10)
        self.path_plan_pub = rospy.Publisher('/gi/navi_path_plan',MarkerArray,queue_size=10)
        # 偏行角度改变
        self.yaw_target_pub = rospy.Publisher('gi/set_pose/orientation', Float32, queue_size=10)
        #t2 = thread.start_new_thread(self.Dstar_thread, ())

        '''
        ros publishers
        '''
        # rc重载发布
        self.rc_override_pub = rospy.Publisher('mavros/rc/override', OverrideRCIn, queue_size = 10)
        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        print("Px4 Controller Initialized!")
        #self.keep_navigating()

        self.path = []
        self.new_path = []
        self.path_prune = PathPruning(obstacle_distance=8)
        time.sleep(2)

    def plot_polygon(self):
        self.driver.simple_plot( )

    '''
    Navigating thread    
    '''
    def keep_navigating(self):
        self.error_yaw = self.current_yaw -180 
        
        # com = Commander()
        t2 = threading.Thread(target=self.plot_polygon)
        t2.start()
        t3 = threading.Thread(target=self.find_path)
        t3.start()
        # scan_environment = True/
        # 将第一次的位置记录到路径中
        self.path.append(self.current_pos)
        while self.mavros_state == "OFFBOARD" and not(rospy.is_shutdown()):
            # self.driver.set_current_pos(self.current_pos)
            # 先扫描一下附近的情况将周围环境扫描一边在执行任务
            # if scan_environment :
            #     # 旋转一周扫描周围环境
            #     except_yaw = 90
            #     com.turn(except_yaw)
            #     # 等待其偏移到指定角度
            #     self.wait_yaw(except_yaw)
            #     except_yaw = 180
            #     com.turn(except_yaw)
            #     # 等待其偏移到指定角度
            #     self.wait_yaw(except_yaw)
            #     except_yaw = -90
            #     com.turn(except_yaw)
            #     # 等待其偏移到指定角度
            #     self.wait_yaw(except_yaw)
            #     except_yaw = 0
            #     com.turn(except_yaw)
            #     # 等待其偏移到指定角度
            #     self.wait_yaw(except_yaw)
            #     scan_environment = False


            # current_pos = self.get_current_pose()
            
            if not self.new_path:
                    # TODO set status
                    print ('No path found!')
                    # self.do_hover()  # TODO
                    time.sleep(0.1)  # TODO
            else:

                m_arr = MarkerArray()
                marr_index = 0
                # 将路径发布出去可以在rviz 中看到
                for next_move in self.new_path:
                    point = self.dg.discrete_to_continuous_target((next_move[0],next_move[1],self.ground_h))
                    mk = Marker()
                    mk.header.frame_id="map"
                    mk.action=mk.ADD
                    mk.id=marr_index
                    marr_index+=1
                    mk.color.r = 1.0
                    mk.color.a = 1.0
                    mk.type=mk.CUBE
                    mk.scale.x = 0.3
                    mk.scale.y = 0.3
                    mk.scale.z = 0.3
                    mk.pose.position.x = point[0]
                    mk.pose.position.y = point[1]
                    mk.pose.position.z = point[2]
                    m_arr.markers.append(mk)
                self.path_plan_pub.publish(m_arr)
                # 执行移动
                for next_move in self.new_path:
                    self.path_plan_pub.publish(m_arr)
                    # if self.navi_task_terminated():
                        # break

                    # print ('current_pos:', current_pos)
                    # 判断新路径中第一个点与 上一路径转向角度是否大于90度，若大于90度则不使用，使用向量点乘，大于0满足条件，小于0不满足
                    # if len(self.path)>1 and self.vector_dot_product(self.path[-2],self.path[-1],self.path[-1],next_move) <0 :
                    #     print ("跳过")
                    #     continue 


                    next_pos = next_move
                    # 设置需要移动的点位
                    relative_pos = (next_pos[0] - self.current_pos[0], next_pos[1] - self.current_pos[1],0)
                    # print ('next_move : ', next_move)
                    # print ("relative_move : ", relative_pos)
                    # print ("next_pose: ", next_pos)

                    # 判断下一个点会不会在障碍物内
                    # if not self.driver.algo.is_valid(next_pos, self.driver.get_obstacles_around()):
                    #     print ('Path not valid!')
                    #     break

                    #axis transform
                    # relative_pos_new = (-relative_pos[0], -relative_pos[1], relative_pos[2])

                    #self.controller.mav_move(*relative_pos_new,abs_mode=False) # TODO:fix this.
                    # print ('mav_move() input: relative pos=',next_pos)

                    # 目标点转向角度为
                    angle =None
                    vector  = (float(self.new_path[-1][0] - self.current_pos[0]),float(self.new_path[-1][1]-self.current_pos[1]))

                    # print ("vector",vector)

                    if vector[0] !=0:
                        angle = math.atan(vector[1]/vector[0]) *180 / math.pi #计算出角度
                        if vector[0] <0 :
                            angle +=180
                    else :
                        if vector[1] <0:
                            angle = -90
                        elif vector[1] >0:
                            angle =90
                    # print ("angle ",angle)
                    # 设置偏移
                    if angle :
                        self.controller._turn_relative_to_takeoff_abs(angle)
                        self.wait_yaw(angle)

                    # 移动开始  
                    self.controller.mav_move(*self.dg.discrete_to_continuous_target((next_pos[0],next_pos[1],self.ground_h)),abs_mode=True)  # TODO:fix this.

                    time.sleep(2)
                    predict_move = (self.current_pos[0] + relative_pos[0], self.current_pos[1] + relative_pos[1],
                                    self.ground_h + relative_pos[2])
                    # print ("predict_move : ", predict_move)
                    
                    # 等待移动成功
                    # 当当前位置与期望位置的向量与期望位置向量与当前位置的向量夹角小于90度时认为完成当前任务执行下次任务
                    while  self.distance(self.current_pos,next_pos) >1 and self.vector_dot_product(next_pos,self.current_pos,self.path[-1],next_pos)<0:
                        # print ("next_pos : ", next_pos)
                        # print (" self.current_pos : ",  self.current_pos)
                        time.sleep(0.1)
                    # 将新路径保存。


                    if self.path[-1] != self.current_pos:
                        self.path.append(self.current_pos)
                    

                    # print ("self.path",self.path)
                    break
                    #  判断路径有无交叉
                    # if not self.algo.path_is_valid(self.collear_check_path, self.driver.get_obstacles_around()):
                    #     print ('Path conflict detected!')
                    #     break
    def vector_dot_product(self,vector1_start,vector1_end,vector2_start,vector2_end):
        '''
        计算两向量之间的点乘
        vector1_start : 第一个向量的起点
        vector1_end : 第一个向量的末点
        vector2_start : 第二个向量的起点
        vector2_end : 第二个向量的末点
        '''
        vector1 = (vector1_end[0]-vector1_start[0],vector1_end[1]-vector1_start[1])
        vector2 = (vector2_end[0]-vector2_start[0],vector2_end[1]-vector2_start[1])
        dot_product = vector1[0]*vector2[0]+vector1[1]*vector2[1]
        return dot_product


    def wait_yaw(self,except_yaw):
        '''
        等待偏移
        '''

        print ("except_yaw",except_yaw)
        start_time = time.time()

        while self.driver.angle_diff(self.current_yaw, except_yaw) >2 :
            time.sleep(0.1)
            print (time.time()-start_time)
                    # 获取高于某高度的地图使其化为2d 的保留在，x,y 去除高度信息
            # map_2d = self.driver.get_octomap_in__h(self.ground_h)
            if time.time()-start_time>20:
                
                break
                # 根据2d 的地图提取地图边界
                # self.driver.get_octomap_alpha_shape(map_2d, self.alpha)
            

    def arrival_position(self,current_pos,max_val):
        '''
        判断是否到达上次输入目标附近
        '''
        x = abs(current_pos[0]-self.set_point_pos[0])
        y = abs(current_pos[1]-self.set_point_pos[1])
        if x < max_val and y <max_val:
            return False
        else :
            return True

    def find_path(self):
        '''
        获取船到岸的距离
        '''
        while True :
            try :
                # time1 = time.time()
                # 获取高于某高度的地图使其化为2d 的保留在，x,y 去除高度信息
                map_2d = self.driver.get_octomap_in__h(self.ground_h)

                # time2 = time.time()
                if len (map_2d)>3:
                    # 根据2d 的地图提取地图边界
                    map_bounds = self.driver.get_octomap_alpha_shape(map_2d, self.alpha)

                    # time3 = time.time()

                    if not map_bounds.is_empty:
                        # 根据多边形边界，获取距离当前位置最近的多边形，和最近多边形上距离当前位置最近的点
                        nearest_point = self.driver.get_nearest_points_and_polygon(self.current_pos,map_bounds)

                        # time4 = time.time()

                        current_axis_yaw = float(self.current_yaw)
                        # 由当前位置 和指向角度可以计算得到
                        # time5 = time.time()
                        
                        current_pos_end = (self.current_pos[0]+10*math.cos(current_axis_yaw/180*math.pi),self.current_pos[1]+10*math.sin(current_axis_yaw/180*math.pi))
                        point = list (nearest_point.coords)[0]
                        # 计算出当前岸线是在左侧还是在船右侧
                        self.bank_status = self.driver.yaw_left_or_right(self.current_pos,current_pos_end,point)

                        # time6 = time.time()

                        # 根据岸线在船的左右位置判断，将当前位置与多边形最近点所形成的线段./顺时针或逆时针以1度步进不断旋转到90度，获得直线与所有多边形的交点，将交点连成一个或多个线段。表示岸线

                        path_lines = self.driver.get_polygon_path(self.current_pos,nearest_point,map_bounds,self.bank_status)
                        print path_lines
                        # 将当之间距离很近将路径合并成一条路径，但当路经大于某个值时丢弃当前路径，
                        path  = self.driver.get_path(path_lines,self.current_pos,self.bank_status,self.except_dist,10)

                        if path !=None and len(path) >0:
                            # 移除在同一直线上的路径，并移除掉第一个点
                            self.new_path = self.path_prune.remove_collinear_points(path)

                            self.driver.set_current_pos(self.current_pos,self.new_path) 
                            print ("new_path",self.new_path)
                        # time7 = time.time()
            except Exception as e:
                print ("EXception in find path: ", e)
                traceback.print_exc()
                    # print ("time1:  " ,round((time2-time1),2))
                    # print ("time2:  " ,round((time3-time2),2))
                    # print ("time3:  " ,round((time4-time3),2))
                    # print ("time4:  " ,round((time5-time4),2))
                    # print ("time5:  " ,round((time6-time5),2))
                    # print ("time6:  " ,round((time7-time6),2))
                    # print ("AllTime :  " ,round((time7-time1),2))
                    # print ("
                    # ===============================================" )

    def print_log(self,):
        '''
        打印日志
        '''
        print ("self.yaw",self.current_point_cloud)
        print ("self.right_parallel_yaw",self.bank_parallel_yaw)
        
        # print ("self.right_K",self.right_K)
        print ("self.right_dist",self.bank_dist)
        # print ("ground",self.ground_speed)
        print ("=================================")


    def set_dist_pid_control(self ):
        '''
        根据期望离岸距离 不断调整yaw 到达期望距离
        except_dist 期望到达的距离,
        now_dist 当前距离值,
        parallel_yaw  车身与岸平行时的yaw 值

        rcmsg   设置通道值 
        '''
        self.dist_pid.SetPoint = self.except_dist
        self.dist_pid.update(float(self.bank_dist))
        self.dist_pid.sample_time = 0.1 #采样间隔

        # 根据距离最近的岸在船身哪侧获得期望yaw 值
        if self.bank_status:
            except_yaw = int( self.bank_parallel_yaw - self.dist_pid.output ) # 得到的期望yaw 值
        else :
            except_yaw = int( self.bank_parallel_yaw + self.dist_pid.output ) # 得到的期望yaw 值
        # 期望yaw 应在0-360 之间 
        if except_yaw > 360 :
            except_yaw = except_yaw % 360
        elif except_yaw < 0 :
            except_yaw = except_yaw % 360 + 360

        print ("except_yaw",except_yaw)

        # 根据期望yaw 值不断调整车身与岸距离
        return except_yaw

    def set_yaw_pid_control(self , except_yaw ,rcmsg):
        '''
        根据期望yaw 值调整pwm 到达期望yaw 值
        except_yaw 期望到达的yaw值,
        rcmsg   设置通道值 
        '''
        self.yaw_pid.SetPoint = except_yaw
        self.yaw_pid.yaw_update(float(self.current_yaw)) # 针对0 -360 yaw 值针对 设置pid
        self.yaw_pid.sample_time = 0.1 #采样间隔
        rcmsg.channels[0]= int( 1500 + self.yaw_pid.output )
        # 设定pwm 值最大为2000 最小为1000
        if rcmsg.channels[0] > 2000 :
            rcmsg.channels[0] = 2000
        elif rcmsg.channels[0] < 1000 :
            rcmsg.channels[0] = 1000


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


    # 设置为手动模式
    def manual(self):
        if self.flightModeService(custom_mode='MANUAL'):
            return True
        else:
            print("Vechile MANUAL failed")
            return False
    '''
    move quad in body frame
    '''

    def distance(self, p1, p2):
        x_distance = (p2[0] - p1[0])**2
        y_distance = (p2[1] - p1[1])**2

        return np.sqrt(x_distance + y_distance )


    def remove_collinear_points(self, original_path):

        new_path = []
        print ("original_path length: ", len(original_path))

        length = len(original_path) - 2
        new_path.append(original_path[0])
        # new_path.append(original_path[-1])

        print(original_path)

        for i in range(length):

            distance13 = self.distance(original_path[i+2], original_path[i])
            distance12 = self.distance(original_path[i+1], original_path[i])
            distance23 = self.distance(original_path[i+2], original_path[i+1])

            print("distance13 - distance12 - distance23 :", distance13 - distance12 - distance23 )
            if abs(distance13 - distance12 - distance23)  < 0.001:
                # print ("points collinear")
                continue

            else:
                print(original_path[i+1])
                print("not found collinear point")
                new_path.append(original_path[i+1])

        print("new path length: ", len(new_path))
        print(new_path)

        return new_path


    def terminate_navigating(self):
        #TODO
        pass

    def resume_navigating(self):
        #TODO
        pass

    def do_hover(self):
        #TODO
        pass


    def set_target_postion(self, target_position):
        self.found_path = True
        # 当前位置，将目标位置进行离散
        self.cur_target_position = target_position
        print("Current target position in grid: ", self.cur_target_position)
        #print("Set Current Position to: ", target_position[0], target_position[1], target_position[2])




    def set_vision_target(self):
        self.set_status(status.GOING_TO_VISION_TARGET)
        self.set_target_position(xxxxx) #TODO
        pass

    def navi_task_terminated(self):
        if dist(self.local_pose,self.cur_target_position) <0.25:  #TODO: or stop flag is set.
            return True
        else:
            return False

    '''
    Dstar Thread
    
    def Dstar_thread(self):
        while not rospy.is_shutdown():
            while status!= xxx:# TODO
                next_move = xxx
                return next_move'''

    '''##For test:
        target = [0.5, 0.5, 0.5]
        self.set_target_postion(target)
        pass'''




    '''
    ROS thread
    responsible for subscribers and publishers
    '''
    def ros_thread(self):
        print('ros_thread spawn!!!!')
        self.octomap_msg = None
        
        # subscribers
        self.slam_sub = rospy.Subscriber("/gi/slam_output/pose", PoseStamped, self.slam_pose_callback)
        self.vision_target_sub = rospy.Subscriber("/gi/visual_target/pose", PoseStamped, self.vision_target_callback)
        self.point_cloud_sub = rospy.Subscriber("/camera/left/point_cloud", PointCloud, self.point_cloud_callback)
        # 实时点云数据
        # self.point_cloud_sub = rospy.Subscriber("/cloud_in", PointCloud, self.point_cloud_callback)
        # self.octomap_cells_vis = rospy.Subscriber("/octomap_point_cloud_centers", PointCloud2, self.octomap_update_callback)  #订阅八叉树地图
        self.octomap_cells_vis = rospy.Subscriber("/stereo_camera/points2", PointCloud2, self.octomap_update_callback)  #订阅八叉树地图
        # self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.local_pose_sub = rospy.Subscriber("/mavros/global_position/local", Odometry, self.local_pose_callback)
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)


        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
        # 获取罗盘值
        # self.compass_hdg_sub = rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.compass_hdg_callback) 

        # 获取到设置的坐标以及 yaw
        self.local_target_sub = rospy.Subscriber('mavros/setpoint_raw/local', PositionTarget, self.set_point_callback)

        # publishers
        #self.mavros_control_pub = rospy.Publisher('mavros/Command', Command, queue_size=10)

        self.set_status(status.INITIALIZED)

        rospy.spin()


    '''
    ROS callbacks
    '''
    def set_point_callback(self,msg):
        pose_ = msg.position #TODO:do fusion with visual slam.
        self.set_point_pos = self.dg.continuous_to_discrete((pose_.x,pose_.y,pose_.z))
        # pass

    def imu_callback(self, msg):
        # global global_imu, current_heading
        # self.imu = msg
        self.current_yaw = int (self.q2yaw(msg.orientation)*180/math.pi)
        # self.received_imu = True


    def q2yaw(self, q):

        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad


    def slam_pose_callback(self, msg):
        self.slam_pose = msg

    def vision_target_callback(self, msg):
        self.vision_target = msg
        #print("Received New Vision Target!")

    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode
        #print(msg.mode, type(msg.mode))
        self.navigator_status_pub.publish(self.STATUS)

    def point_cloud_callback(self, msg):
        self.current_point_cloud = msg

    def octomap_update_callback(self, msg):  # as pointcloud2.
        # set 里面为 (x,y,z) 的元组
        # print (msg)
        obs_set = set()
        # a = pc2.read_points(msg)
        for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            #print " x : %f  y: %f  z: %f" % (p[0], p[1], p[2])
            point = self.dg.continuous_to_discrete((p[0],p[1],p[2])) # 将点进行离散化
            #print ('point:',point)
            obs_set.add(point)

        acquired = self.obstacle_set_mutex.acquire(True)  # blocking.  尝试获得锁定。使线程进入同步阻塞状态
        if acquired:
            #print('octomap updated!')
            self.driver.set_obstacle_set(obs_set)  # 设置八叉树
            self.obstacle_set_mutex.release() # 释放锁
            return
        else:
            print ('Lock not acquired!')

    def local_pose_callback(self, msg):
        pose_ = msg.pose.pose.position #TODO:do fusion with visual slam.
        local_pose = self.dg.continuous_to_discrete((pose_.x,pose_.y,pose_.z))
        self.local_pose = (local_pose[0],local_pose[1])
        self.current_pos = self.local_pose
        #print ('local_pose set!!!')

    

    # def compass_hdg_callback(self, msg):
    #     '''
    #     罗盘数据回调函数

    #     msg 罗盘获取到的yaw 值
    #     '''
    #     self.current_yaw = msg.data 


    def get_local_pose(self): # in mavros axis.for command.
        #print ('self.local_pose',self.local_pose)
        return self.local_pose

    def get_current_pose(self): # current pose:slam pose(in world axis)
        return self.get_local_pose() # TODO:do transform T1 ^-1 * T2.



    '''
    helper functions
    '''
    def set_status(self, status):
        self.STATUS = String(status.name)


    '''
    def reachTargetPosition(self, target, threshold = 0.1):

        delta_x = math.fabs(self.local_pose.pose.position.x - target.pos_sp[0])
        delta_y = math.fabs(self.local_pose.pose.position.y - target.pos_sp[1])
        delta_z = math.fabs(self.local_pose.pose.position.z - target.pos_sp[2])

        distance = (delta_x + delta_y + delta_z)

        print("distance: ", distance, "threshold: ", threshold)
        if distance < threshold:
            return True
        else:
            return False
'''

    def setMavMode(self, msg):
        pass


if __name__ == '__main__':
    nav = Navigator()

    #FLU meters.
    # nav.set_target_postion((10, 0, 2.5))
    nav.keep_navigating()

