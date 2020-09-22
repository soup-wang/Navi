#!/usr/bin/env python
# -*- encoding: utf-8 -*-
'''
@文件        :gps_utiles.py
@说明        :gps 使用转换
@时间        :2020/09/04 06:07:52
@作者        :王美汤
@版本        :1.0
'''
import math


def gps_get_angle(Waypoint_start, Waypoint_end):
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


def gps_get_distance(Waypoint_start, Waypoint_end):
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


def gps_to_meter_coord(current_gps, origin_gps=(121, 31)):
    '''
    gps,坐标转化到米坐标系
    current_gps 需要转化的坐标
    origin_gps  坐标零点
    '''
    angle = gps_get_angle(current_gps, origin_gps)*math.pi/180
    distance = gps_get_distance(current_gps, origin_gps)
    mete_cood = (distance*math.cos(angle), distance*math.sin(angle))
    return mete_cood
