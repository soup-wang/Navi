# -*- coding: utf-8 -*-
# 测试 将一堆散点 进行分成多个线段 
import math

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import time
import scipy.stats as st
import pandas as pd

### 最小二乘法 python leastsq###
from scipy.optimize import leastsq


class linesSegment:
    def __init__(self,slope,dist_center,intercept,r_value):
        # 直线的斜率
        self.slope = slope
        # 直线到中心点的距离
        self.dist_center = dist_center

        self.intercept = intercept
        # r_value 存储
        self.r_value = r_value

    def __str__(self):
        return '(linesSegment: %s, %s,%s)' % (self.slope,self.dist_center,self.r_value)
    __repr__ = __str__
plt.ion()


def segment(data,center_point=(0,0),debug=False,debug_data=[]):
    points = []

    lines =[]

    start_i = 0 
    end_i = 3
    slope  = 0
    intercept = 0
    r_value = 0

    #  R-squared，为相关系数R^2的检验，越接近1则越显著
    r_value_center = 1
    slope_good = 0
    intercept_good = 0
    end_i_good = 0

    diff_r_value = 1
    while end_i <= len(data):

        if (end_i-start_i==2):
            if (data[end_i-1][0]-data[end_i-2][0]) ==0 :
                slope =float("inf")
            else:
                slope = (data[end_i-1][1]-data[end_i-2][1])/(data[end_i-1][0]-data[end_i-2][0])
            intercept = data[end_i-1][1]-slope*data[end_i-1][0]
            r_value = 1.0
        else:
            #  只能计算三个以上的值
            slope, intercept, r_value, p_value, std_err = st.linregress(data[start_i:end_i])
        # print ("r_value**2",r_value**2)
        if  abs (r_value**2 - r_value_center)  < diff_r_value:
            diff_r_value = abs (r_value**2 - r_value_center)
            r_value_max = r_value**2
            # print ("r_value_max",r_value_max)
            slope_good =slope
            intercept_good = intercept
            end_i_good = end_i
        if end_i == len(data) :
            # 中心点到直线的距离
            dist_center = abs(center_point[0]*slope_good-center_point[0]+intercept_good)/math.sqrt(slope_good**2+1)
            lines.append(linesSegment(slope,dist_center,intercept,r_value_max))
            if debug:
                if start_i ==0 :
                    points.append((data[start_i][0],slope_good*data[start_i][0]+intercept_good))
                points.append((data[end_i_good-1][0],slope_good*data[end_i_good-1][0]+intercept_good))

            start_i = end_i_good -1
            end_i = start_i + 2
            if len(data) -start_i < 3 :
                end_i = start_i + 1
            r_value_max = 0
            diff_r_value = 1
            # print ("std_err: ",std_err)
        end_i +=1
    # 是否进行调试
    if debug :
        plt.cla()
        c = []
        d = []
        for item in points:
            c.append(item[0])
            d.append(item[1])
        # plt.scatter(c, d, alpha=0.5, marker=",")
        plt.plot(c, d, "g-", linewidth=2.0)
        data.append(center_point)
        a = []
        b = []
        for item in data:
            a.append(item[0])
            b.append(item[1])
        plt.scatter(a, b, alpha=0.5, marker=",")

        a = []
        b = []
        for item in debug_data:
            a.append(item[0])
            b.append(item[1])
        plt.scatter(a, b, alpha=0.5, marker=">")
        # plt.plot(a, b, "r-", linewidth=2.0)
        #     # 显示图形
        # plt.show()
          # # 暂停
        print("lines",lines)
        
        plt.pause(0.05)
    return lines


def segment2(data,center_point=(0,0),debug=False,debug_data=[]):
    points = []

    lines =[]

    start_i = 0 
    end_i = 3
    good_i = 0

    start_slope  = 0
    end_slope  = 0
    start_intercept = 0
    end_intercept = 0
    start_r_value = 0
    end_r_value  = 0

    #  R-squared，为相关系数R^2的检验，越接近1则越显著
    r_value_center = 1
    start_slope_good = 0
    end_slope_good = 0
    start_intercept_good = 0
    end_intercept_good = 0
    start_r_value_good = 0
    end_r_value_good  = 0

    # end_i_good = 0

    # diff_r_value = 1
    while end_i < len(data):

        if (len(data)-end_i==2):
            if (data[end_i-1][0]-data[end_i-2][0]) ==0 :
                end_slope =float("inf")
            else:
                end_slope = (data[end_i-1][1]-data[end_i-2][1])/(data[end_i-1][0]-data[end_i-2][0])
            end_intercept = data[end_i-1][1]-end_slope*data[end_i-1][0]
            end_r_value = 1.0
        else:
            #  只能计算三个以上的值
            start_slope, start_intercept, start_r_value, p_value, std_err = st.linregress(data[start_i:end_i])
            end_slope, end_intercept, end_r_value, p_value, std_err = st.linregress(data[end_i:len(data)])

        if end_r_value**2 >0.5  and end_r_value**2 + start_r_value**2 > r_value_center:
            r_value_center = end_r_value**2 + start_r_value**2
            start_slope_good = start_slope
            end_slope_good = end_slope
            start_intercept_good = start_intercept
            end_intercept_good = end_intercept
            start_r_value_good = start_r_value**2
            end_r_value_good = end_r_value**2
            good_i = end_i

            # print ("std_err: ",std_err)
        end_i +=1
    start_dist_center = abs(center_point[0]*start_slope_good-center_point[0]+start_intercept_good)/math.sqrt(start_slope_good**2+1)
    end_dist_center = abs(center_point[0]*end_slope_good-center_point[0]+end_intercept_good)/math.sqrt(end_slope_good**2+1)
   
    lines.append(linesSegment(start_slope_good,start_dist_center,start_intercept_good, start_r_value_good))
    lines.append(linesSegment(end_slope_good,end_dist_center,end_intercept_good,end_r_value_good))
   
    # 是否进行调试
    if debug :
        points.append(data[start_i])
        points.append((data[good_i][0],data[good_i][0]*start_slope_good+start_intercept_good))
        points.append(data[end_i-1])
        plt.cla()
        c = []
        d = []
        for item in points:
            c.append(item[0])
            d.append(item[1])
        # plt.scatter(c, d, alpha=0.5, marker=",")
        plt.plot(c, d, "g-", linewidth=2.0)
        data.append(center_point)
        a = []
        b = []
        for item in data:
            a.append(item[0])
            b.append(item[1])
        plt.scatter(a, b, alpha=0.5, marker=",")

        a = []
        b = []
        for item in debug_data:
            a.append(item[0])
            b.append(item[1])
        plt.scatter(a, b, alpha=0.5, marker=">")
        # plt.plot(a, b, "r-", linewidth=2.0)
        #     # 显示图形
        # plt.show()
          # # 暂停
        print("lines",lines)
        
        plt.pause(0.05)
    return lines


def error(p,x,y):
    '''
    # p是个数组，表示所有参数!!!
    ### 定义误差函数，拟合y=kx+b，p[0]表示k，p[1]表示b
    ### 法线式 y*sin(theta)+x*cos(theta) = dist
    '''
    #  #x、y都是列表，故返回值也是个列表
    return y*math.sin(p[0])+x*math.cos(p[0])-p[1]

def segment_Trig(data,center_point=(0,0),debug=False):
    '''
    使用三角函数进行拟合，可以拟合出垂直于x 轴的值
    '''
    xdata=[]
    ydata=[]
    for i in data :
        xdata.append(i[0])
        ydata.append(i[1])

    ###采样点(Xi,Yi)###
    Xi=np.array(xdata)
    Yi=np.array(ydata)

    ###主函数从此开始###
    # 可能是使用梯度下降法而非矩阵运算，因此需要给定初始参数p0
    p0=[0,1]
    Para=leastsq(error,p0,args=(Xi,Yi)) #把error函数中除了p以外的参数打包到args中
    theta = Para[0][0]
    dist = Para[0][1]
    # 求误差方差
    err = np.var(error(Para[0],Xi,Yi))

    if theta != 0 :
        intercept = dist/math.sin(theta)
        dist = point_line_dist(center_point,-1/math.tan(theta),intercept)
    else :
        intercept = float("inf")
    # print("error",error(Para[0],Xi,Yi))
    if debug:
        print("theta=",theta*180/math.pi,'\n',"dist=",dist)
        plt.axis([-20,0,0,20])
        plt.scatter(Xi,Yi,color="red",label="Sample Point",linewidth=3) #画样本点
        if theta != 0:           
            x=np.linspace(-100,10,10)
            y=dist/math.sin(theta)-x/math.tan(theta)
        else:
            x = dist
            y = np.linspace(-100,10,10)
        plt.plot(x,y,color="orange",label="Fitting Line",linewidth=2) #画拟合直线
        plt.legend()
        plt.show()

    return linesSegment_Trig(theta,intercept,dist,err) 
class linesSegment_Trig:
    def __init__(self,theta,intercept,dist,err):
        # 直线的斜率
        self.theta = theta
        # 直线到中心点的距离
        
        self.dist = dist

        self.intercept = intercept
        # r_value 存储
        self.err = err

    def __str__(self):
        return '(linesSegment: %s, %s,%s)' % (self.theta,self.dist,self.err)
    __repr__ = __str__

def segment_Trig_2(data,center_point=(0,0),debug=False):
    points = []

    lines =[]

    start_i = 0 
    end_i = 2
    good_i = 0
    #  记录两个方差最好的结果
    err_good = float("inf")

    start_line  = None
    start_line_good = None

    end_line  = None
    end_line_good = None

    while end_i < len(data)-1:

        #  计算出角度方差，方差应越接近与0 拟合结果越好
        start_line = segment_Trig(data[start_i:end_i],center_point)
        end_line = segment_Trig(data[end_i:len(data)],center_point)

        # 保存最好的方差和数据
        if start_line.err + end_line.err < err_good:
            err_good = start_line.err + end_line.err
            start_line_good = start_line
            end_line_good = end_line
            good_i = end_i

            # print ("std_err: ",std_err)
        end_i +=1
    # 若角度等于0 则与x 轴垂直
 
   
    lines.append(start_line_good)
    lines.append(end_line_good)
   
    # # 是否进行调试
    # if debug :
    #     points.append(data[start_i])
    #     points.append((data[good_i][0],data[good_i][0]*start_slope_good+start_intercept_good))
    #     points.append(data[end_i-1])
    #     plt.cla()
    #     c = []
    #     d = []
    #     for item in points:
    #         c.append(item[0])
    #         d.append(item[1])
    #     # plt.scatter(c, d, alpha=0.5, marker=",")
    #     plt.plot(c, d, "g-", linewidth=2.0)
    #     data.append(center_point)
    #     a = []
    #     b = []
    #     for item in data:
    #         a.append(item[0])
    #         b.append(item[1])
    #     plt.scatter(a, b, alpha=0.5, marker=",")
        
    #     plt.pause(0.05)
    return lines,good_i

def point_line_dist (point,slope,intercept):
    '''
    点到直线的距离
    point: 点(x,y)
    slope:  直线斜率    
    intercept:  直线截距
    '''
    return abs(point[0]*slope-point[0]+intercept)/math.sqrt(slope**2+1)

if __name__ == "__main__":
    test_data = [
        (-20,0),(-19.8,0.2),(-19.5,0.8),(-19.3,0.9),(-19,1.5),(-18.5,2),(-18.2,2.5),(-17.8,3),(-17,4),(-16.6,4.5),(-16.5,5),
        (-16.4,4.8),(-15.8,4.2),(-15.3,3.5),(-15.1,3),(-15,2.8),
        (-14.9,3),(-14.5,3.4),(-14,3.8),(-13.6,4.2),(-13.3,4.8),(-12.6,5.3),(-12.1,5.8),(-11.5,6.5),
        (-11.2,6.5),(-11,6.4),(-10.3,6.46),(-9.5,6.3),(-8.4,6.2),(-8,6.19),(-7.3,6.12),(-6.5,6.1),(-5.4,6.01),(-4,5.99),(-3,6.01),
        (-4,6.4),(-5,7),(-6.3,9),(-8,10),(-9,12),(-11.2,15),(-12,17),(-13.2,19.7),(-14.7,22),(-15.9,23),(-17,25.5),(-18.3,27),(-19.5,30),
        (-8,38),(-6,42),
    ]
    # test_data = [
    #     (-20,0),(-19.8,0.2),(-19.5,0.8),(-19.3,0.9),(-19,1.5),(-18.5,2),(-18.2,2.5),(-17.8,3),(-17,4),(-16.6,4.5),(-16.5,5),
    #     (-16.4,4.8),(-15.8,4.2),(-15.3,3.5),(-15.1,3),(-15,2.8),
    # ]
    

    x= st.linregress([(1,1),(2,2),(3,3)])
    # 循环
    for index in range(50):
        start_time = time.time()
        points = segment(test_data,debug=True)
        print("time_",time.time()-start_time)
        print ("points",points)

    