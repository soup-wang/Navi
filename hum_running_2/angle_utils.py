# -*- coding: utf-8 -*-


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


if __name__ == "__main__":
    am = angle_min_different_0_360(320,180)
    print("am",am)