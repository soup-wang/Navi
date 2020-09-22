# -*- coding: utf-8 -*-
import math
# 计算角度使角度一直在0-360 之间循环

def angle_addition_0_360(angle1,angle2):
    '''
    计算两个角度和
    '''
    return (angle1+angle2)%360

def angle_different_0_360(angle1,angle2):
    '''
    计算两个角度差
    '''
    return (angle1-angle2)%360

def angle_min_different_0_360(angle1,angle2):
    '''
    计算两个角度之间的角度,这个值小于180度
    '''
    diff = angle_different_0_360(angle1,angle2)
    if diff > 180 :
        diff = angle_different_0_360(angle2,angle1)
    return diff

def angle_median_0_360(angle1,angle2):
    '''
    计算两个角度中间的角度值
    '''
    diff = angle_different_0_360(angle1,angle2)
    angle_median = angle_addition_0_360(angle2,diff/2.0)
    if diff > 180 :
        diff = angle_different_0_360(angle2,angle1)
        angle_median = angle_addition_0_360(angle1,diff/2.0)
    return angle_median
 
def calc_angles_stddev(data_list):    
    '''
    计算角度的标准偏差
    '''
    m_sin, m_cos = calc_mean_sin_cos(data_list)
    # 计算标准偏差
    stddev = math.sqrt(-math.log(m_sin**2 + m_cos**2)) 
    return stddev
 
def calc_angles_mean1(data_list):
    '''
    Mardia坐标中的平均角度计算
    '''
    m_sin, m_cos = calc_mean_sin_cos(data_list)
    # 计算平均角度
    mean_angle = math.atan(m_sin/m_cos)*(180/math.pi)  #calculate mean angle
    # arctan1 = math.atan(m_sin/m_cos)*(180/math.pi)
    if (m_cos >= 0 ):
        mean_angle = mean_angle
    elif (m_cos < 0):
        mean_angle = mean_angle + 180
    if mean_angle < 0 :
        mean_angle = 360 + mean_angle  # 将 (-pi/2, 0)这一段的值转换成正值
    return mean_angle
 
def calc_angles_mean2(data_list):
    '''
    指南针坐标中的平均角度计算
    '''
    lst = []
    for elem in data_list:  # 坐标系转换，将罗盘坐标系的角度值，转换到Mardia的坐标系下
        if elem > 90:
            elem = 450 - elem
        else:
            elem = 90 - elem
        lst.append(elem)
    
    m_sin, m_cos = calc_mean_sin_cos(lst) # 算 sin 和 cos 均值
 
    mean_angle = math.atan(m_sin/m_cos)*(180/math.pi)  # 计算角度均值，
    # arctan2 = math.atan(m_sin/m_cos)*(180/math.pi)
 
    if (m_cos >= 0 ): # 依据Mardia的方法，均值转换
        mean_angle = mean_angle
    elif (m_cos < 0):
        mean_angle = mean_angle + 180
 
    if mean_angle < 0:  # 上面计算得到的均值范围(-pi/2, 3pi/2)，这里将负值转换成正值
        mean_angle = 360 + mean_angle
 
    if mean_angle < 90:  # 将均值再转换回风向本身坐标系下
        mean_angle = 90 - mean_angle
    else:
        mean_angle = 450 - mean_angle
 
    return mean_angle

def calc_mean_sin_cos(data_list):
    '''
    计算一组角度的平均正弦和余弦
    '''
    m_sin = 0
    m_cos = 0
    for data_elmt in data_list:
        m_sin += math.sin(math.radians(data_elmt))
        m_cos += math.cos(math.radians(data_elmt))
    m_sin /= len(data_list)  #average of list sin
    m_cos /= len(data_list)
 
    return m_sin, m_cos
 

if __name__ == "__main__":
    am = angle_min_different_0_360(320,180)
    print("am",am)