# -*- coding: utf-8 -*-
# 引入ros的Python包
import rospy
from mavros_msgs.msg import State, OverrideRCIn, RCIn, RCOut, VFR_HUD, HomePosition
from mavros_msgs.srv import CommandBool,  SetMode
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
from visualization_msgs.msg import *
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from hum_running_msgs.msg import HumRunning,Config
# 引入点云信息
import sensor_msgs.point_cloud2 as pc2

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

import matplotlib.pyplot as plt
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

import gps_utiles



class Px4Controller:

    def __init__(self):
        rospy.init_node("test")
        self.config = Config()
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
        kml_path = current_path + "/kml/" + self.config.kml_name

        print(kml_path)

        # 读取kml 文件
        self.kml_dict = {}
        self.read_kml_file(kml_path)
     

        self.dg = DiscreteGridUtils.DiscreteGridUtils()

        # 启动一个线程定时更新配置文件
        Thread(target=self.read_config, args=(config_path,)).start()
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
        # self.left_dist_queue = Queue()
        # self.left_parallel_yaw_queue = Queue()
        # 均值滤波
        self.left_dist_queue = []
        self.left_parallel_yaw_queue = []
        # 当前gps 位置
        self.current_gps = None
        # 返航点位置
        self.home_gps = None

        # 当前位置距离航点的距离
        self.current_waypoint_dist = 0
        self.current_heading = 1
        self.arm_state = False
        # 手动模式切换状态
        self.manual_state = False
        self.frame = "BODY"
        self.state = None
        #
        self.left_parallel_yaw = None
        # 点云数据
        self.point_cloud = None

        # 设置直线斜率

        self.cloud_points = []

        self.num = 1
        # 存放gps 历史路径
        self.gps_queue = Queue()

        '''
        ros publishers
        '''
        # rc重载发布
        self.rc_override_pub = rospy.Publisher(
            'mavros/rc/override', OverrideRCIn, queue_size=10)
        self.marker_pub = rospy.Publisher("/track/heading", Marker, queue_size=10)
        # 路径发布
        self.path_pub = {}
        for key in self.kml_dict:
            # 发布航线
            self.path_pub[key] = rospy.Publisher("/track/"+key, Path, queue_size=10)

        self.shore_line_pub = rospy.Publisher("/track/Shoreline" , Path, queue_size=10)
        self.gps_path_pub = rospy.Publisher("/track/gps_path" , Path, queue_size=10)
        self.hum_msg_pub = rospy.Publisher("/track/hum_msg" , HumRunning, queue_size=10)
        '''
        ros subscribers
        '''
        # 获取imu 信号
        rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)

        # 获取遥控器输入信号
        rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_in_callback)
        #  获取遥控最终输出信号
        rospy.Subscriber("/mavros/rc/out", RCOut, self.rc_out_callback)
        # 获取地速
        rospy.Subscriber("/mavros/vfr_hud", VFR_HUD, self.vfr_hub_callback)
        # 获取GPS位置
        rospy.Subscriber("/mavros/global_position/global",
                         NavSatFix, self.GPS_callback)
        # 获取apm 连接 状态
        rospy.Subscriber("/mavros/state", State, self.State_callback)
        # 获取点云数据
        rospy.Subscriber("/voxel_grid/points", PointCloud2,
                         self.point_cloud_callback)
        # 获取home 返航点位置
        rospy.Subscriber("/mavros/home_position/home",
                         HomePosition, self.home_GPS_callback)

       
        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy(
            '/mavros/set_mode', SetMode)

        print("Px4 Controller Initialized!")


    def start(self, debug=False):
        rcmsg = OverrideRCIn()
        rcmsg.channels = [65535, 65535, 65535,
                          65535, 65535, 65535, 65535, 65535]
        # 设置程序状态，0 为未设置，1为遥控，2 为延边状态
        first_cortron_state = 0
        # 航点索引
        waypoint_index = 0
        bridge_time = 0
        # 0 向桥首个航点驶入
        # 1 进入直行状态
        # 2 过桥完成,前往过桥后与当前航向不超过70度的航点
        bridge_status = 3
        hum_msg = HumRunning()
        

        while True:
            # 切入解锁
            # self.log.logger.info("arm_state:" + str(self.arm_state))
            # self.log.logger.info("manual_state:" + str(self.manual_state))
            if self.arm_state and self.manual_state and (rospy.is_shutdown() is False):
                if self.current_gps != None and len(self.rc_in) > 2 and self.rc_in[5] > 1500:
                    if first_cortron_state != 2:
                        rcmsg.channels[2] = 1700

                        self.log.logger.info("延边模式")
                        # 首次切入延边状态进入前往地一个航点
                        first_cortron_state = 2
                        waypoint_index = 0
                        waypoint = self.current_gps

                    # 判断当前位置与航点距离多少
                    self.current_waypoint_dist = gps_utiles.gps_get_distance(
                        self.current_gps, waypoint)

                    self.current_heading = gps_utiles.gps_get_angle(
                        self.current_gps, waypoint)
                    hum_msg.currenrHeading = self.current_heading
                    hum_msg.currenrYaw = self.yaw

                    if bridge_status == 3:
                        min_bridge_waypoints = self.get_min_bridge()
                        # 获取距离当前位置与最近桥的距离以及桥本身航点
                        if len(min_bridge_waypoints) > 0:
                            bridge_heading = gps_utiles.gps_get_angle(
                                min_bridge_waypoints[0], min_bridge_waypoints[-1])
                            bridge_distance = gps_utiles.gps_get_distance(
                                min_bridge_waypoints[0], min_bridge_waypoints[-1])
                            self.log.logger.info(
                                "开始进行过桥,桥长" + str(bridge_distance) + "米 ,桥起始点: "+str(min_bridge_waypoints[0]))
                            # 满足过桥行动
                            bridge_time = 0
                            bridge_status = 0

                        elif self.current_waypoint_dist < self.config.min_waypoint_distance:

                            if waypoint_index < len(self.kml_dict["left"]):
                                waypoint = self.kml_dict["left"][waypoint_index]
                                waypoint_index += 1
                                self.log.logger.info(
                                    "前往第"+str(waypoint_index)+"个航点:"+str(waypoint))

                            elif waypoint_index == len(self.kml_dict["left"]):
                                self.log.logger.info(
                                    "到达第"+str(waypoint_index)+"个航点:"+str(waypoint))
                                self.log.logger.info("任务结束")
                                waypoint_index += 1

                    # 若当前位置距离桥第一个点小于最小航点距离则进行
                    elif bridge_status == 0:
                        current_bridge_heading = gps_utiles.gps_get_angle(
                            self.current_gps, min_bridge_waypoints[0])
                        current_bridge_distance = gps_utiles.gps_get_distance(
                            self.current_gps, min_bridge_waypoints[0])

                        self.set_yaw_pid_control(
                            current_bridge_heading, rcmsg ,hum_msg)
                        if current_bridge_distance < self.config.min_waypoint_distance:
                            bridge_status = 1
                            start_time = time.time()

                    elif bridge_status == 1:

                        self.set_yaw_pid_control(bridge_heading, rcmsg ,hum_msg)
                        bridge_time = time.time()-start_time
                        if bridge_time > (bridge_distance +5) / self.config.ground_speed:
                            self.log.logger.info(
                                "已通过桥,耗时"+str(bridge_time)+"秒,桥末尾点: "+str(min_bridge_waypoints[-1]))
                            # 确定通过桥梁
                            bridge_status = 2

                    elif bridge_status == 2:
                        if waypoint_index < len(self.kml_dict["left"])-1 and angle_utils.angle_different_0_360(self.current_heading, self.yaw) > 90:
                            waypoint = self.kml_dict["left"][waypoint_index]
                            self.log.logger.info(
                                "跳过第"+str(waypoint_index)+"个航点:"+str(waypoint))
                            waypoint_index += 1
                        else:
                            self.log.logger.info(
                                "前往第"+str(waypoint_index)+"个航点:"+str(waypoint))
                            bridge_status = 3

                    # self.log.logger.info( "current_heading: " + str(self.current_heading))

                    # 判断点云是否符合沿岸要求，符合要求是时进行沿岸行走,不符合要求航向向下个航点行驶,或者进入调试模式下不进入延边模式
                    if len(self.cloud_points) > 5 and not self.config.yaw_PID_debug and bridge_status == 3:
                        self.get_dist_parallel_yaw()
                        if self.left_dist != None and self.left_parallel_yaw != None and self.left_dist > 0.5:
                            hum_msg.leftDist = self.left_dist
                            hum_msg.leftParallelYaw = self.left_parallel_yaw
                            
                            # 根据与岸的距离不断调整 yaw 是其不断逼近想要的距离
                            self.set_dist_pid_control(
                                self.config.except_dist, self.left_dist, self.left_parallel_yaw, rcmsg,hum_msg)
                        else:
                            self.left_dist = None 
                            self.left_parallel_yaw = None
                    else:
                        self.left_dist = None 
                        self.left_parallel_yaw = None

                        # 向航点航向行驶
                        self.set_yaw_pid_control(self.current_heading, rcmsg,hum_msg)
                        time.sleep(0.1)
                # 如果切换到遥控模式
                elif len(self.rc_in) > 2 and self.rc_in[5] < 1500:

                    if first_cortron_state != 1:
                        self.log.logger.info("遥控模式")
                        first_cortron_state = 1
                        # rcmsg.channels[2] = 1500


                    rcmsg.channels[2] = 65535
                    # rcmsg.channels[0] = self.rc_in[1]
                    rcmsg.channels[0] = 65535


                    time.sleep(0.1)
            # self.log.logger.info("rcmsg:" + str(rcmsg))
            hum_msg.config=self.config
            self.rc_override_pub.publish(rcmsg)
            self.hum_msg_pub.publish(hum_msg)

    def get_min_bridge(self):

        # 当船距离桥小于最小值时认为需要进行过桥
        current_bridge_distance = self.config.min_bridge_distance
        min_bridge_waypoints = []
        bridge_heading = None
        # 判断桥的的位置距离有多远,返回船最近距离桥的最近点且与下个航点的的航向,与当前位置与桥的航向超过180度时,忽略
        for key in self.kml_dict:
            if "bridge" in key:
                # self.log.logger.info("bridge")
                # 获取桥的中线位置,当得到船距离桥只有15m 时进入过桥模式固定yaw值

                start_bridge_distance = gps_utiles.gps_get_distance(
                    self.current_gps, self.kml_dict[key][0])
                end_bridge_distance = gps_utiles.gps_get_distance(
                    self.current_gps, self.kml_dict[key][-1])

                if start_bridge_distance < current_bridge_distance:
                    current_bridge_distance = start_bridge_distance
                    min_bridge_waypoints = self.kml_dict[key]
                    bridge_heading = gps_utiles.gps_get_angle(
                        self.kml_dict[key][0], self.kml_dict[key][-1])

                if end_bridge_distance < current_bridge_distance:
                    current_bridge_distance = end_bridge_distance
                    min_bridge_waypoints = self.kml_dict[key][::-1]
                    bridge_heading = gps_utiles.gps_get_angle(
                        self.kml_dict[key][-1], self.kml_dict[key][0])

        if bridge_heading == None or angle_utils.angle_min_different_0_360(bridge_heading, self.yaw) > 70:
            min_bridge_waypoints = []

        # 返回当前位置与桥的航点位置
        return min_bridge_waypoints

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
        self.config.except_dist = config["except_dist"]  # 期望与岸的距离为 0.7
        # 设置转向 pid设置，调试时应先调整转向pid 保证能很好地转到某个yaw值
        self.config.yaw_PID_debug = config["yaw_PID_debug"]  # 是否调试yawPID
        # 调试Yaw PID时应固定一个yaw看是否能 很好地转到需要方向
        self.config.yaw_PID_yaw = config["yaw_PID_yaw"]
        self.config.course_range = config["course_range"]  # 设置航向范围，在原始航点最大偏转30度
        self.config.yaw_P = config["yaw_P"]
        self.config.yaw_I = config["yaw_I"]
        self.config.yaw_D = config["yaw_D"]
        # 设置yaw pid设置
        self.yaw_pid = PID.PID(P=self.config.yaw_P, I=self.config.yaw_I, D=self.config.yaw_D)

        # 设置距离 pid设置，
        self.config.dist_P = config["dist_P"]
        self.config.dist_I = config["dist_I"]
        self.config.dist_D = config["dist_D"]
        # 设置dist_pid pid设置
        self.dist_pid = PID.PID(P=self.config.dist_P, I=self.config.dist_I, D=self.config.dist_D)

        # 多少值参与均值滤波 越大表示反应越缓慢
        self.config.filter_N = config["filter_N"]
        # 是否画出岸与船的关系
        self.config.plot_debug = config["plot_debug"]
        # 是否写出日志
        self.config.log_debug = config["log_debug"]
        # 是否打印日志
        self.config.print_log_debug = config["print_log_debug"]

        # kml 文件名
        self.config.kml_name = config["kml_name"]

        # 最小航点距离，当船距离航点距离最小距离时，认为已经到达航点,单位m
        self.config.min_waypoint_distance = config["min_waypoint_distance"]

        # 执行过桥最近距离,小于此距离执行过桥动作
        self.config.min_bridge_distance = config["min_bridge_distance"]

        # 地面速度
        self.config.ground_speed = config["ground_speed"]

    def avg_filter(self, avg_list, value, filter_N,last_value):
        '''
        均值滤波函数
        queue : 储存滤波队列
        value : 滤波的值
        filter_N : n个平均值参与滤波
        '''
        # queue.put(value)
        avg_list.append(value)
        # if last_value != None:
        #     avg_list.append(last_value) 
        # 均值滤波
        # if queue.qsize() == filter_N:
        # print avg_list
        if len (avg_list)>= filter_N:

            # list_sort = list(queue.queue)
            avg_list.sort()
            avg_list.pop(0)
            avg_list.pop(-1)

            # # 弹出队列中数据
            # queue.get()
            # list_sort.sort()
            # 去除一个最大值一个最小值 求平均获得最终结果
            # return round(((sum(list_sort)-list_sort[0]-list_sort[-1])/(filter_N-2)), 6)
            return round((sum(avg_list)/len (avg_list)), 6)
        else:
            return last_value

    def set_dist_pid_control(self, except_dist, now_dist, parallel_yaw, rcmsg ,hum_msg):
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
        self.set_yaw_pid_control(except_yaw, rcmsg ,hum_msg)

    def except_yaw_range(self, except_yaw):
        '''
        根据期望角度范围设置期望角度
        '''
        
        current_heading_up = self.current_heading + self.config.course_range
        current_heading_down = self.current_heading - self.config.course_range

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

    def set_yaw_pid_control(self, except_yaw, rcmsg,hum_msg):
        '''
        根据期望yaw 值调整pwm 到达期望yaw 值
        except_yaw 期望到达的yaw值,
        rcmsg   设置通道值 
        '''
        hum_msg.exceptYaw = except_yaw

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

    def point_cloud_callback(self, msg):
        '''
        获取点云信息 
        '''

        self.cloud_points = list(pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True))

    def home_GPS_callback(self, msg):
        '''
            获取home_GPS位置 
        '''
        self.home_gps = (msg.geo.longitude, msg.geo.latitude)

      

    def get_dist_parallel_yaw(self):
        # 第一象限数
        quadrant_1 = set()
        quadrant_2 = set()
        quadrant_3 = set()
        quadrant_4 = set()
        all_points = []
        center_point = (0, 0)
        for point in self.cloud_points:
            h_top = 1
            h_down = 0.2
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
        paths = cen.contour()
        if len(paths) > 1 :
            line = LineSegment.segment_Trig(
                paths, center_point=center_point)
            # 当线段拟合程度不够时使用双折线拟合
            if line.err > 0.01:
                lines, good_i = LineSegment.segment_Trig_2(
                    paths, center_point=center_point)
                # 双折线中选择k 值为负值且拟合程度大于0.75的线
                # 次选拟合程度最大的
                if good_i > len(paths)-good_i and lines[0] != None:
                    line = lines[0]
                elif lines[1] != None:
                    line = lines[1]
            self.num += 1
            paths.append((0, self.num))
            # 均值滤波
            self.left_dist = self.avg_filter(
                self.left_dist_queue, line.dist, self.config.filter_N,self.left_dist)
            # 岸线的平行的角度
            
            min_x = float("inf")
            min_y = float("inf")
            max_x = -float("inf")
            max_y = -float("inf")
            for point in paths:
                if point[0] > max_x:
                    max_x = point[0]
                if point[0] < min_x:
                    min_x = point[0]
                if point[1] > max_y:
                    max_y = point[1]
                if point[1] < min_y:
                    min_y = point[1]
            if line.theta != 0:
                slope = -1/math.tan(line.theta)
                path_list =[(min_x,min_x*slope+line.intercept),(max_x,max_x*slope+line.intercept)]
            else :
                path_list = [(-line.dist,min_y),(-line.dist,max_y)]
            path = self.create_path(path_list,frame_id="world")
            self.shore_line_pub.publish(path)
            deflection =  line.theta*180/math.pi
            left_parallel_yaw = (self.yaw - deflection) % 360
            # 均值滤波
            self.left_parallel_yaw = self.avg_filter(
                self.left_parallel_yaw_queue, left_parallel_yaw, self.config.filter_N,self.left_parallel_yaw)

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
        marker = self.create_marker() 
        self.marker_pub.publish(marker)
          # 将航线发布出来
        if self.home_gps != None:
            for key in self.kml_dict:
                path = self.create_path(self.kml_dict[key])
                self.path_pub[key].publish(path)

    def  create_path(self,path_list,frame_id = "map"):
        '''
        创建path 
        '''
        path = Path()
        path.header.frame_id= frame_id
        path.header.stamp =  rospy.Time.now()
        for point in path_list:
            # 将gps 转为m 的点
            if frame_id == "map":
                point = gps_utiles.gps_to_meter_coord(point,self.home_gps)
            pose = self.create_pose(point,frame_id)
            path.poses.append(pose)
        
        return path
            
    
    def create_pose (self,point,frame_id = "map"):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp =  rospy.Time.now()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    def create_marker(self):
        '''
        创建，marker
        '''
        marker = Marker()

        #指定Marker的参考框架
        marker.header.frame_id = "base_link"
        
        #时间戳
        marker.header.stamp = rospy.Time.now()
        
        #ns代表namespace，命名空间可以避免重复名字引起的错误
        marker.ns = "boat_yaw"
        
        #Marker的id号
        marker.id = 0
        
        #Marker的类型，有ARROW，CUBE等
        marker.type = Marker.ARROW
        
        #Marker的尺寸，单位是m
        marker.scale.x = 1
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        #Marker的动作类型设置箭头
        marker.action = Marker.ADD
        
        #Marker的位置姿态
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        orientation = tf.transformations.quaternion_from_euler( 0, 0, math.pi/2)
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]
        
        #Marker的颜色和透明度
        marker.color.r = 0.8
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.9
        
        #Marker被自动销毁之前的存活时间，rospy.Duration()意味着在程序结束之前一直存在
        return marker

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
        # self.config.ground_speed = msg.groundspeed
        pass

    def GPS_callback(self, msg):
        '''
        获取GPS 信息
        '''
        self.current_gps = (msg.longitude, msg.latitude)
        self.GPS_status = msg.status.status
        if self.current_gps !=None and self.home_gps !=None:
            
            self.gps_queue.put(self.current_gps)
            if self.gps_queue.qsize()>200:
                self.gps_queue.get()
            gps_path = self.create_path(list(self.gps_queue.queue))
            self.gps_path_pub.publish(gps_path)


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
