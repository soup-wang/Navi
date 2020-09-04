# -*- coding: utf-8 -*-
# 已知一个点，和一堆点集。求出在这些点集中距离该点最近的轮廓线
# 四象限还可以通过四个线程进行多线程优化

import math

import numpy as np
import matplotlib.pyplot as plt
import matplotlib
import time

from shapely.geometry import LineString


class CenterPointEdge:

    def __init__(self, points, center_point, aerfa):
        '''
        points : 二维点集数据，格式：[(x0,y0),(x1,y1),(x2,y2)...]
        center_point : 环绕的中心点
        aerfa ： 每次旋转的角度
        '''
        self.classification = points
        self.center_point = center_point
        self.aerfa = aerfa

    def contour(self):
        '''
        计算出环绕中心位置的轮廓点
        '''
        # 便利出所有列表数据
        classification = self.classification
        # 点
        points = []

        # 角度遍历次数
        rotationTimes = int(90/self.aerfa)

        for item in range(len(classification)):
            pointRelationships = classification[item]
            # 若没有值则结束当前循环
            if len(pointRelationships) == 0:
                continue

            # 索引至为零
            index = 0

            # 遍历每次偏转一个角度，在此角度范围内找最近的点
            # 每次区间为[tan(i*aerfa),tan((i+1)*aerfa))
            indexOff = False

            for i in range(rotationTimes):

                k0 = math.tan(i*self.aerfa/180.0 * math.pi)
                k1 = math.tan((i + 1)*self.aerfa/180.0 * math.pi)
                # 第二四象限 k 值为负值
                if item == 1 or item == 3:
                    k0 = math.tan((i - rotationTimes)*self.aerfa/180.0 * math.pi)
                    k1 = math.tan((i - rotationTimes + 1) *
                                  self.aerfa/180.0 * math.pi)

                # 设置索引开关，开关闭合代表该象限所有点数据全部遍历结束
                if indexOff:
                    break

                pointRelationship = None
                for j in range(len(pointRelationships)):
                    # 当在一三象限k 值为正值
                    if item == 0 or item == 2:
                        # 当k 值在 k0 与k1 之间时
                        if (i == rotationTimes and pointRelationships[index].k >= k0) or (i != rotationTimes and pointRelationships[index].k >= k0 and pointRelationships[index].k < k1):
                            # 当pointRelationship 为None 时直接赋值

                            if pointRelationship == None or pointRelationship.distent > pointRelationships[index].distent:
                                pointRelationship = pointRelationships[index]
                                index += 1
                            else:
                                index += 1
                        else:
                            break
                    else:
                        # 当k 值在 k0 与k1 之间时
                        if (i == 0 and pointRelationships[index].k < k1) or (i != 0 and pointRelationships[index].k >= k0 and pointRelationships[index].k < k1):
                            if pointRelationship == None or pointRelationship.distent > pointRelationships[index].distent:
                                pointRelationship = pointRelationships[index]
                                index += 1
                            else:
                                index += 1
                        else:
                            break
                    # 如果索引等于于len(pointRelationships)，所有数据已经遍历 索引开关闭合
                    if index == len(pointRelationships):
                        indexOff = True
                        break
                if pointRelationship != None:
                    points.append(pointRelationship.point)

        return points

    def remove_collinear_points(self, original_path):
        '''
        删除重复点
        '''
        # 删除重复点
        new_path = []
        length = len(original_path)
        # 第一的点距离当前位置较近不考虑
        new_path.append(original_path[0])

        # 判断三点之间的距离若满足小于0.001认为在同一条直线上
        for i in range(length)[2:]:

            distance13 = self.points_dist(original_path[i], original_path[i-2])
            distance12 = self.points_dist(original_path[i-1], original_path[i-2])
            distance23 = self.points_dist(original_path[i], original_path[i-1])

            if abs(distance13 - distance12 - distance23) < 0.0005:
                continue

            else:
                new_path.append(original_path[i-1])

        new_path.append(original_path[-1])

        return new_path

    def test(self):
        points = self.contour()
        # print (points)
        return points
        # pass

    def Point_set_classification(self):
        '''
        将点集数据进行归类存储
        '''
        PointsClassification = []
        # 第一象限数
        quadrant_1 = []
        quadrant_2 = []
        quadrant_3 = []
        quadrant_4 = []

        for point in self.points:
            dist = self.points_dist(point, self.center_point)
            k = (point[1] - self.center_point[1]) / \
                (point[0] - self.center_point[0])
            # print("k : ", k)
            # 判断点云以中心点为原点划分象限判断其在在哪个象限内
            if point[1] - self.center_point[1]> 0:
                if point[0]-self.center_point[0] > 0:
                    # 第一象限
                    quadrant_1.append(
                        PointRelationship(dist, 1, k, point))
                else:
                    # 第二象限
                    quadrant_2.append(
                        PointRelationship(dist, 2, k, point))
            else:
                if point[0]-self.center_point[0] > 0:
                    # 第四象限
                    quadrant_4.append(
                        PointRelationship(dist, 4, k, point))
                else:
                    # 第三象限
                    quadrant_3.append(
                        PointRelationship(dist, 3, k, point))
        PointsClassification.append(
            sorted(quadrant_1, key=lambda item: item.k))
        PointsClassification.append(
            sorted(quadrant_2, key=lambda item: item.k))
        PointsClassification.append(
            sorted(quadrant_3, key=lambda item: item.k))
        PointsClassification.append(
            sorted(quadrant_4, key=lambda item: item.k))

        return PointsClassification

    def points_dist(self, point1, point2):
        '''
        计算两点之间的距离
        point1 : (x,y)
        '''
        return math.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)


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


if __name__ == "__main__":

 # 打开交互模式
    plt.ion()

    # 循环
    for index in range(50):
        # 清除原有图像
        plt.cla()

        # 生成测试数据
        N = 100000
        x = np.random.randn(N)
        y = np.random.randn(N)

        # y = x *y
        # x = y * x
        points = []

        for i in range(N):
            # if x[i]-2 < 0 and  (x[i]-2)**2 -(y[i]+1)**2  >=1 or y[i]-2*(x[i] -2 )>2:
            #     points.append((x [i], y[i]))

            # if y[i]-2*x[i] > 0.7*2 :
            #     points.append((x[i], y[i]))

            
            if x[i] >0.51 or x[i] <-0.5:
                points.append((x[i], y[i]))

            # print (points)
        center_point = (0,0)
        aerfa = 5
        cen = CenterPointEdge(points, center_point, aerfa)
        start_time = time.time()
        test = cen.test()

        # line = LineString(test)
        # lines = line.parallel_offset(distance=0.1, side="left").coords[:]

        # paths = cen.remove_collinear_points(lines)

        end_time = time.time()

        print("实际用时：", end_time-start_time)

        a = []
        b = []
        print("实际点集数量", len(points))
        for item in points:
            a.append(item[0])
            b.append(item[1])
        plt.scatter(a, b, alpha=0.5)

        c = []
        d = []
        for item in test:
            c.append(item[0])
            d.append(item[1])
        # plt.scatter(c, d, alpha=0.5, marker=",")
        plt.plot(c, d, "g-", linewidth=2.0)

        # c = []
        # d = []
        # for item in paths:
        #     c.append(item[0])
        #     d.append(item[1])
        # # plt.scatter(c, d, alpha=0.5, marker=",")
        # plt.plot(c, d, "b--", linewidth=2.0)

        plt.scatter([center_point[0]], [center_point[1]], marker=">")

        # 暂停
        plt.pause(2)

    # 关闭交互模式
    plt.ioff()

    # 显示图形
    plt.show()
