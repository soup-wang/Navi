# 引入队列
from queue import Queue

from pykml import parser

import matplotlib.pyplot as plt

import math

dict = {}


def read_kml_file(kml_path):
    # 读取kml文件
    with open(kml_path, 'r', encoding='UTF-8') as f:
        doc = parser.parse(f).getroot()

    Placemarks = doc.Document.Folder.Placemark

    for Placemark in Placemarks:

        plot_list_x = []
        plot_list_y = []
        gps_queue = Queue()
        All_GPS = Placemark.LineString.coordinates.text.replace(
            '\n', '').replace('\t', '').replace('0 ', '')
        gps_list = All_GPS.split(",")
        for i in range(0, int((len(gps_list)-1)/2)):
            gps_queue.put((float(gps_list[2*i]), float(gps_list[2*i+1])))
            plot_list_x.append(float(gps_list[2*i]))
            plot_list_y.append(float(gps_list[2*i+1]))

        dict[Placemark.name.text] = gps_queue

        # plt.cla()
        # # plt.axis([121, 122, 31.125, 31.13])
        # plt.plot(plot_list_x, plot_list_y, "g-", linewidth=2.0)
        # plt.show()


def lat_and_lon_get_angle1(Waypoint_start, Waypoint_end):
    '''
    计算两个经纬度之间的方位角
    Waypoint_start : 起始点(经度，纬度)
    Waypoint_end ： 指向末尾点(经度，纬度)
    '''

    y = math.sin(Waypoint_end[0]-Waypoint_start[0]) * math.cos(Waypoint_end[1])
    x = math.cos(Waypoint_start[1])*math.sin(Waypoint_end[1]) - math.sin(Waypoint_start[1]) * \
        math.cos(Waypoint_end[1])*math.cos(Waypoint_end[0]-Waypoint_start[0])
    brng = math.atan2(y, x)*180/math.pi

    if brng < 0:
        brng = brng + 360
    return brng


def lat_and_lon_get_distance(Waypoint_start, Waypoint_end):
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


def except_yaw_range(except_yaw):
    '''
    根据期望角度范围设置期望角度
    '''

    current_heading = 50

    current_heading_up = current_heading + 30
    current_heading_down = current_heading - 30

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


if __name__ == "__main__":
    kml_path = "C:\\Users\\tangtang\\Desktop\\Navi\\hum_running_2\\岸线.kml"

    read_kml_file(kml_path)
    right_queue = dict["right00"]
    Waypoint_start = right_queue.get()
    Waypoint_end = right_queue.get()
    brng = lat_and_lon_get_angle1(Waypoint_start, Waypoint_end)
    print("brng", brng)

    print("anglle", -367 % 360)

    except_yaw = 270

    print("except_yaw", except_yaw_range(except_yaw))

    dist = lat_and_lon_get_distance(Waypoint_start, Waypoint_end)
    print("dist", dist)
