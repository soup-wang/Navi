#encoding=utf-8
from astar import A_star
from multiprocessing import Queue,Process
from time import sleep
import rospy


import sys
from descartes import PolygonPatch
import matplotlib.pyplot as plt
import alphashape
import math

from shapely.geometry import Polygon,LineString,JOIN_STYLE,MultiPolygon,Point
from shapely.ops import cascaded_union,nearest_points
from shapely.affinity import rotate ,scale

from time import time
class Driver:
    def __init__(self): #init_pos, end_pos,command_queue,returnval_queue):
        #self.algo = A_star(end_pos)
        #self.init_pos = init_pos
        #self.current_pos = init_pos
        #self.end_pos = end_pos
        #self.command_queue = command_queue
        #self.returnval_queue = returnval_queue
        #self.sleeptime = 0.1  #TODO:delete this.
        self.TIME_DELAY_THRESHOLD = 500 # TODO: fix this.

        self.path_lines =[]
        self.path = []
        self.new_path =[]
        self.current_pos = [0,0]
        self.current_pos_end = [0,0]
        #to keep a obstacle set buffer.
        self.obstacle_set = set()
        self.octomap_in_h_set = set()
        self.obs_set_last_update_time = rospy.rostime.get_time()
        self.bank_muiltpolygon = MultiPolygon()

    def set_current_pos(self,current_pos,new_path):
        self.current_pos = current_pos
        self.new_path = new_path

    def set_obstacle_set(self,obstacle_set):
        self.obstacle_set = obstacle_set
        self.obs_set_last_update_time = rospy.rostime.get_time()

    
    def get_obstacles_around(self):
        current_time = rospy.rostime.get_time()
        if current_time - self.obs_set_last_update_time > self.TIME_DELAY_THRESHOLD:
            print ("Warning:Buffer timeout!Delay:",current_time-self.obs_set_last_update_time)
        return self.obstacle_set

    def get_octomap_in__h(self,h):
        '''
        获取高于某个高度的投影 obstacles 地图 转化为（x ,y ） 代表的地图
        h : z 轴值大于h 点   类型：任意常数

        return : 2d的地图散点   类型： [(x,y),(x,y)...]
        '''
        point_cloud2d =set()
        obstacle_set = self.get_obstacles_around()
        for item in obstacle_set:
            # 打印下当前的高度层数据为
            # print ("当前高度",item[2])
            # 获取某高度层上距离为5的一部分数据
            if h +10 >item[2] > h :
                point_cloud2d.add((item[0],item[1]))

        self.octomap_in_h_set = point_cloud2d
        return point_cloud2d
    
    def get_point_yaw(self,current_pos,current_pos_end,points):
        '''
            判断两点与当前位置点的方向距离当前方向哪个更近
            若向量用坐标表示，a=(x1,y1), b=(x2,y2)，则，a.b=(x1x2+y1y2)。

            |a|=√(x1^2+y1^2)，|b|=√(x2^2+y2^2)。

            将这些代人公式(Ⅰ),得到：

            cos θ =(x1x2+y1y2)/[√(x1^2+y1^2)*√(x2^2+y2^2)]

            上述公式是以空间三维坐标给出的，令坐标中的z=0,则得平面向量的计算公式。两个向量夹角的取值范围是：[0,π]。

            夹角为锐角时，cosθ>0；

            夹角为钝角时,cosθ<0。

            返回两个点之间的距离
            point1  (x,y)
            point2  (x,y)

            为防止计算平方时float溢出，做如下转换
            x_dist = x2-x1
            y_dist = y2-y1
             √(x_dist²+y_dist²) = √((x_dist/y_dist)²+1) * abs(y_dist)
        '''
        x = current_pos_end[0] - current_pos[0]
        y = current_pos_end[1] - current_pos[1]

        x1 = points[0][0] - current_pos[0]
        y1 = points[0][1] - current_pos[1]

        x2 = points[1][0] - current_pos[0]
        y2 = points[1][0] - current_pos[1]

        theta1 = math.acos((x1*x+y1*y)/((x1**2+y1**2)**0.5*(x**2+y**2)**0.5))
        theta2 = math.acos((x2*x+y2*y)/((x2**2+y2**2)**0.5*(x**2+y**2)**0.5))

        if theta1 < theta2:
            return points[0]
        else:
            return points[1]
        # x_dist = abs(point2[0]-point1[0]) 
        # y_dist = abs(point2[1]-point1[1])
        # # if y_dist = 0 
        # # inner = x_dist / y_dist 
        # return  (x_dist**2+y_dist**2)**0.5 

    def get_octomap_alpha_shape (self, map_2d, alpha ):
        '''
        获取某高度以上的轮廓多边形

        map_2d : 2d 的散点地图  类型： [(x,y),(x,y)...]
        alpha : 控制生成的边界  类型：任意常数

        return : 多个各个边界点  类型：[((x,y),(x,y)..),((x,y),(x,y)..)...]
        它的原理可以想象成一个半径为α 的圆在点集 s 外滚动，当α足够大时，
        这个圆就不会滚到点集内部，其滚动的痕迹就是这个点集的边界线。因此，当α
        值很小，则每个点都是边界；如果α 很大（α →∞ ）时，则求出的边界线为
        点集 s 的凸包。
        '''

        points  = list(map_2d)
        # 当获取到的点云数据大于2 时才能求他的边缘
        if len (points) > 2 :
        # print (points)
            # 获取点云的边缘多边形
            
            time1 = time()
            try:
                alpha_shape = alphashape.alphashape(points, alpha)
            except Exception:
                print ("异常")
                return self.bank_muiltpolygon 

            
            time2 = time()

            print ("alpha_shape_time1:  " ,round((time2-time1),2))
            polygon_list = []
            if type(alpha_shape)==Polygon :
                polygon_list.append(alpha_shape)
                
            else :
                for polygon in alpha_shape:
                    polygon_list.append(polygon)

            for p in  self.bank_muiltpolygon:
                polygon = Polygon(p)
                polygon_list.append(polygon)
            # 将多个多边形进行融合
            polygons = cascaded_union(polygon_list)
            # 曲线平滑处理
            polygons = polygons.simplify(0.6,preserve_topology=False)
            if type(polygons)==Polygon :
                self.bank_muiltpolygon = MultiPolygon([polygons])
            else :
                self.bank_muiltpolygon = polygons

            # print (self.bank_muiltpolygon)
            return self.bank_muiltpolygon
        else:
            return self.bank_muiltpolygon
    def get_nearest_points_and_polygon(self, point,polygons):
        '''
        根据多个多边形和一个坐标，返回距离坐标点最近的多边形，以及最近多边形中最近的点

        @params point 坐标点 
        @params polygons 多个多边形

        @return 返回多边形和最近的点

        '''
        point = Point (point)
        # min_dist =None
        # min_dist_polygon = None
        # for polygon in polygons:
        #     dist = point.distance(polygon)
        #     if min_dist == None or dist <min_dist:
        #         min_dist = dist
        #         min_dist_polygon = polygon
        
        # 从最小多边形获取最近点
        nearest_point = nearest_points(point,polygons)[1]

        # 将最小多边形和最近点返回
        return   nearest_point

    def get_polygon_path(self,current_pos, nearest_point,polygons ,bank_status):
        '''
        根据岸线在船的左右位置判断，将当前位置与多边形最近点所形成的线段顺时针或逆时针以1度步进不断旋转到90度，
        获得直线与所有多边形的交点，将交点连成一个或多个线段。表示岸线
        
        为保证每次偏转角度所去点的距离近似。计算偏转角度
        使用公式求出下个偏转角度

        # start_angle = 距离/

        next_angle = arctan(n*tan(start_angle)) - last_angle

        bank_status : True 代表左侧,顺时针旋转 0 - 90度  ，False 代表右侧 逆指针旋转 0 - -90度
        '''
        path_lines =[]

        line = LineString([current_pos,nearest_point])

        for polygon in polygons:
            path_line =[]
            start_angle = 10*math.pi/180
            for a in range(0 ,20):
                next_angle = math.atan(a*math.tan(start_angle))*180/math.pi 
                # print ("next_angle ",next_angle)
                # 将线旋转一定角度
                a_line = None
                if bank_status :
                    a_line =rotate(line, -next_angle,origin = current_pos)
                else :
                    a_line =rotate(line, next_angle,origin = current_pos)
                # 将直线延长原来的10倍


                n =10
                a_line_zoom = scale(a_line,xfact=n,yfact=n,origin=current_pos)
                # 求放大之后的直线与多边形 的交线
                polygon_intersection= polygon.intersection(a_line_zoom)
                if not polygon_intersection.is_empty:
                # 根据交线获得距离当前位置最近的点
                    path_point = nearest_points(Point(current_pos),polygon_intersection)

                    path_line.append(list(path_point[1].coords)[0])


            if len (path_line )>0:
                path_lines.append(path_line)

        self.path_lines = path_lines
        return path_lines

    def get_path(self,path_lines,current_pos,bank_status,expct_dist,max_dist):
        '''
        # 将当之间距离很近将路径合并成一条路径，但当路经大于某个值时丢弃当前路径，
        '''
        point = Point(current_pos)
        min_dist =None
        min_dist_line = None
        for line in path_lines:
            if len(line) > 1:
                line_string = LineString(line)
            else :
                line_string = Point(line[0])

            dist = point.distance(line_string)
            if min_dist == None or dist <min_dist:
                min_dist = dist
                min_dist_line = line
        
        print path_lines
        # 移除掉最近距离所在的线
        path_lines.remove(min_dist_line)
        if len(min_dist_line) > 1:
            min_dist_line_string = LineString(min_dist_line)
        else :
            min_dist_line_string = Point (min_dist_line[0])

        # 将距离相近的直线拼成一条路径
        for line in path_lines:
            if len(line) > 1:
                line_string = LineString(line)
            else :
                line_string = Point(line[0])

            if line_string.distance(min_dist_line_string) <= max_dist:
                min_dist_line.extend(line)
        # print ("min_dist_line: ",min_dist_line)
        if len (min_dist_line)>1:
            path_line = LineString(min_dist_line)
            # 判断左右将直线往外扩展并光滑
            path = None
            if  bank_status:
                path = path_line.parallel_offset(expct_dist,side="right",join_style=JOIN_STYLE.mitre)
            else:
                path = path_line.parallel_offset(expct_dist,side="left",join_style=JOIN_STYLE.mitre)

            # path = list(LineString(path).simplify(0.3,preserve_topology=False).coords) 
            # path_set = set()
            # try :
            #     for point in list(path.coords):
            #         path_set.add((int(point[0]),int(point[1])))
            # except Exception:
            #     print ("异常")

            
            self.path = list(path.coords)
          
            return self.path 


    def simple_plot(self):
        print (self.current_pos)
        """
        simple plot
        """
        # 生成画布
        plt.figure(figsize=(10, 10), dpi=80)

        # 打开交互模式
        plt.ion()

        # 循环
        while True:
            # 清除原有图像
            plt.cla()

            
            plt.xlim(-150, 200)
            plt.ylim(-200, 150)

            # 当前位置指向
            x = [self.current_pos[0]]
            y = [self.current_pos[1]]
            plt.plot( x,y , ">", color='y', linewidth=2.0)

            x = [self.current_pos_end[0]]
            y = [self.current_pos_end[1]]
            plt.plot( x,y , ".", color='r', linewidth=2.0)
            # print (self.bank_muiltpolygon)

            # 多边形形状
            for polygon in self.bank_muiltpolygon:
                x = []
                y = []
                for point in list( polygon.exterior.coords):
                    x.append(point[0])
                    y.append(point[1])
            
                plt.plot(x, y, "g-", color= 'b', linewidth=2.0)

            # # 多边形路径
            # for line in self.path_lines:
            #     x = []
            #     y = []
            #     for point in line:
            #         x.append(point[0])
            #         y.append(point[1])
            
            #     plt.plot(x, y, ".",color ='r' , linewidth=2.0)
            
            # # 点云数据
            # x = []
            # y = []
            # for point in list(self.octomap_in_h_set):
                
            #     x.append(point[0])
            #     y.append(point[1])
            
            # plt.plot(x, y, ".",color ='b' , linewidth=2.0)


            # 路径数据
            x = []
            y = []
            for point in self.new_path:
                
                x.append(point[0])
                y.append(point[1])
            
            plt.plot(x, y, ">",color ='r' , linewidth=2.0)
            # 暂停
            plt.pause(1)


    def polygon_area(self,points):
        """
        返回多边形面积
        points : 散点组成的多边形  类型：[(x,y),(x,y)...]

        return : 返回多边形的面积
        """
        point_num = len(points)
        if(point_num < 3): return 0.0
        s = points[0][1] * (points[point_num-1][0] - points[1][0])
        #for i in range(point_num): # (int i = 1 i < point_num ++i):
        for i in range(1, point_num): # 有小伙伴发现一个bug，这里做了修改，但是没有测试，需要使用的亲请测试下，以免结果不正确。
            s += points[i][1] * (points[i-1][0] - points[(i+1)%point_num][0])
        return abs(s/2.0)

    def get_point_2_polygon_dist(self,point,boundarys):
        '''
        获取点到多边形的最近距离和多个多边形中顶点在多边形中的索引
        point : 点  类型：(x,y)
        boundrays : 多个多边形散点集合 类型： [((x,y),(x,y)..),((x,y),(x,y)..)...]

        return : 返回各个多边形距离点的距离和索引 和各个多边形的面积[(area,dist,index,polygon_index),(area,dist,index,polygon_index)...]
        '''
        points_a_d_i_p = []
        bound_index = 0
        for polygon in boundarys :

            # 计算点的到多边形的距离和 索引
            nearest_dist = ((polygon[0][0]-point[0])**2 + (polygon[0][1]-point[1])**2)**0.5
            nearest_point_index = 0
            for index in range (1,len(polygon)):
                dist = ((polygon[index][0]-point[0])**2 + (polygon[index][1]-point[1])**2)**0.5
                if dist < nearest_dist :
                    nearest_dist = dist
                    nearest_point_index = index
            # 计算多边形的面积
            area = self.polygon_area(polygon)

            points_a_d_i_p.append((area,nearest_dist,nearest_point_index,bound_index))
            bound_index += 1

        return points_a_d_i_p

    def get_current_pos_nearest_points(self,nearest_point_index,boundary_points ,n ,bank_status):
        '''
        返回障碍物边界距离当前位置最近的点 的前后n 个点的集合

        nearest_point_index : 最近点索引  类型：常数
        
        boundary_points : 障碍物轮廓集合   类型：[(x,y),(x,y)...]
        
        n: 最近点向当前方向前方取值 n 个点  类型： n > 0 且为整数

        bank_status : 判断最近点在现在方向的左侧还是右侧 ，bank_status 为True 左侧逆时针取值，bank_status 为False 右侧顺时针，d=0中间顺时针

        ruturn : 返回多个点的集合  类型 ： [[x1,x2,x3...],[y1,y2,y3...]]
        '''
        nearest_points_x=[]
        nearest_points_y=[]
        for i in range (0,n):
            if bank_status : # 顺时针
                index = nearest_point_index + i 
            else :  #逆时针
                index = nearest_point_index - i 

            # 环装结构
            if index <= 0 :
                index = len(boundary_points) + index -1
            elif index >= len (boundary_points)-1:
                index = index - len(boundary_points) + 1
            nearest_points_x.append(boundary_points[index][0])
            nearest_points_y.append(boundary_points[index][1])
        return [nearest_points_x,nearest_points_y]

    def get_points_slope(self , points ):
        '''
        获取几个散点斜率,


        points ： 多组点数 类型：[[x1,x2,x3...],[y1,y2,y3...]]
    
        return : 直线斜率

        '''
        points_x = points[0]
        points_y = points[1]

        
        # points_x = [1,2,4,5,6]
        # points_y = [2,2,2,2,2]

        avg_x =  sum(points_x)/len(points_x) 
        avg_y =  sum(points_y)/len(points_y) 
        
        em = 0 
        ez = 0 
        for i in range(0,len(points_x))  :
            em += ((points_x[i] - avg_x) * (points_y[i] - avg_y)) 
            ez += (points_x[i] - avg_x)**2

        # 当ez 等于0 则得到的曲线为 竖直的直线 计算距离直线上第一个点的距离为x 加上距离
        if ez == 0 :
            return None
        # 当em 等于0 所得直线为平行的直线
        elif em == 0 and ez !=0 :
            return 0
        else :
            return em/ez # 计算与原始直线垂直的直线斜率


        
    def isPointinPolygon(self,point, rangelist): 
        '''
        判断点是否在多边形内
        point : 点 [x,y]
        rangelist : 多边形 [(x,y),(x,y)] 
        '''
        # 判断是否在外包矩形内，如果不在，直接返回false

        lnglist = []
        latlist = []
        for i in range(len(rangelist)-1):
            lnglist.append(rangelist[i][0])
            latlist.append(rangelist[i][1])
        # print(lnglist, latlist)
        maxlng = max(lnglist)
        minlng = min(lnglist)
        maxlat = max(latlist)
        minlat = min(latlist)
        # print(maxlng, minlng, maxlat, minlat)
        if (point[0] > maxlng or point[0] < minlng or
            point[1] > maxlat or point[1] < minlat):
            return False
        count = 0
        point1 = rangelist[0]
        for i in range(1, len(rangelist)):
            point2 = rangelist[i]
            # 点与多边形顶点重合
            if (point[0] == point1[0] and point[1] == point1[1]) or (point[0] == point2[0] and point[1] == point2[1]):
                # print("在顶点上")
                return True
            # 判断线段两端点是否在射线两侧 不在肯定不相交 射线（-∞，lat）（lng,lat）
            if (point1[1] < point[1] and point2[1] >= point[1]) or (point1[1] >= point[1] and point2[1] < point[1]):
                # 求线段与射线交点 再和lat比较
                point12lng = point2[0] - (point2[1] - point[1]) * (point2[0] - point1[0])/(point2[1] - point1[1])
                # print(point12lng)
                # 点在多边形边上
                if (point12lng == point[0]):
                    print("点在多边形边上")
                    return False
                if (point12lng < point[0]):
                    count +=1
            point1 = point2
        # print(count)
        if count%2 == 0:
            return False
        else:
            return True


    def diff_yaw (self,start_yaw,end_yaw):
        '''
        计算两个角度的差值，使其归于 0-360 之间
        '''
        diff_yaw = start_yaw - end_yaw
        if diff_yaw > 360 :
            diff_yaw = diff_yaw % 360
        elif diff_yaw < 0 :
            diff_yaw = diff_yaw % 360 + 360

        return diff_yaw


    def geo_angle_2_axis_angle (self,geo_angle,error_yaw):
        '''
        将地理角度值 （0-360）计算偏转 转化为坐标系角度（-180-0-180）

        geo_angle : 将地理角度值
        error_yaw : 将地理角度值与坐标系角度值偏差值

        return : 坐标系角度值
        '''
        # 将当前角度校正为坐标轴的角度表示值
        geo_angle += error_yaw
        if geo_angle < 0:
            geo_angle = 360+geo_angle
        elif geo_angle > 360 :
            geo_angle = geo_angle -360 

        # 校正之后转换为 -180-0 -180
        axis_angle = geo_angle - 180
        return axis_angle

    def axis_angle_2_geo_angle (self,axis_angle,error_yaw):
        '''
        将坐标系角度（-180-0-180）转化为 计算偏转 地理角度值 （0-360）

        axis_angle : 坐标系角度
        error_yaw : 地理角度值与坐标系角度值偏差值

        return : 地理角度值

        '''
        geo_angle = axis_angle + 180
        # geo_angle -=error_yaw

        if geo_angle < 0:
            geo_angle = 360+geo_angle
        elif geo_angle > 360 :
            geo_angle = geo_angle -360 

        return geo_angle

    def angle_diff(self,angle_1,angle_2):
            '''

                计算两个角度的角度之间的夹角

                angle_1 : 角度值 取值：-180-0 -180
                angle_2
            '''
            angle_diff = abs (angle_1-angle_2)
            if angle_diff > 180 :
                angle_diff = 360 - angle_diff 

            return angle_diff


    def get_dist_piont ( self,start_point,k,d , current_pos, move_dist ):
        '''
        在一条直线上 ，根据一点获取距离d 的另一点的坐标

        start_point: 起始点 (x1,y1)
        k ： 直线的斜率
        d : 两点之间的距离
        current_pos : (x0,y0)

        Line 1: ax1 + by1 + c1 = 0
        a = k 
        b = -1
        c = y1-k*x1

        d(Line1,Line2)=∣c1−c2∣/ (a**2+b**2)**0.5
        c2 = c1-d * (a**2+b**2)**0.5 = y1 -k*x1 -d *(k**2+1)**0.5
        c2 = d * (a**2+b**2)**0.5 -c1 = d *(k**2+1)**0.5 -y1+k*x1

        判断点距离哪个直线更近
        line1 = kx -y +c2
        d = |kx0-y0+c|/(k**2+1)**0.5

        使用距离最短的获得最终的直线
     


        公式 ： 
            将方程最后化简为医院二次方程,最后得出化简结果
            ax**2 + mx + c =0
        '''
        # 当直线的斜率为无穷大，直线与y 轴平行
        if (k==None):
            # 计算当前点点到两直线的距离
            d1 =abs (current_pos[0] - (start_point[0]-d))
            d2 =abs (current_pos[0] - (start_point[0]+d))

            if d1 < d2:
                point1 = (int(start_point[0]-d),int(start_point[1]+move_dist))
                point2 = (int(start_point[0]-d),int(start_point[1]-move_dist))
                return (point1,point2)
            else:
                point1 = (int(start_point[0]+d),int(start_point[1]+move_dist))
                point2 = (int(start_point[0]+d),int(start_point[1]-move_dist))
                return (point1,point2)
        # 当直线斜率为0时，直线与x 轴平行
        elif k==0:
            # 计算当前点点到两直线的距离
            d1 = abs(current_pos[1] - (start_point[1]-d))
            d2 = abs(current_pos[1] - (start_point[1]+d))

            if d1 < d2:
                point1 = (int(start_point[0]+move_dist),int(start_point[1]-d))
                point2 = (int(start_point[0]-move_dist),int(start_point[1]-d))
                return (point1,point2)
            else:
                point1 = (int(start_point[0]+move_dist),int(start_point[1]+d))
                point2 = (int(start_point[0]-move_dist),int(start_point[1]+d))
                return (point1,point2)
        else:
            # 计算得到与当前直线距离为d的两个平行线
            c = self.get_dist_lines(d,k,start_point)
            # 计算当前位置到两直线的距离
            d1 = self.get_point_line_dist(current_pos,k,c[0])
            d2 = self.get_point_line_dist(current_pos,k,c[1])

            k1=-1/k # k1为与其垂直的直线
            # 得到与当前直线垂直的直线，并得到距离垂直线为move_dist的直线
            c1 = self.get_dist_lines(move_dist,k1,start_point)


            if d1 < d2:
                point1 = self.get_lines_node(k,k1,c[0],c1[0])
                point2 = self.get_lines_node(k,k1,c[0],c1[1])
                return (point1,point2)
            else: 
                point1 = self.get_lines_node(k,k1,c[1],c1[0])
                point2 = self.get_lines_node(k,k1,c[1],c1[1])
                return (point1,point2)

        




        # a = 1 + (-1/k)**2
        # m = 2*(-1/k)*c - 2*x - 2*(-1/k)*y
        # n = x**2 + y**2 - move_dist**2 - 2*c*y + c**2 

        # end_x1 = int ((-m + ((m**2)-4*a*n)**0.5)/(2*a))
        # end_x2 = int ((-m - ((m**2)-4*a*n)**0.5)/(2*a))

        # end_y1 = int ( (-1/k)*end_x1 + c)
        # end_y2 = int ( (-1/k)*end_x2 + c )

        # return ((end_x1,end_y1),(end_x2,end_y2))
    def get_lines_node(self,k1,k2,c1,c2):
        '''
        计算两条直线的交点
        k1: 斜率
        k2:
        c1: 截据
        c2:

        x = (c2-c1)/(k1-k2)
        y = k1*x+c1
        '''
        x = (c2-c1)/(k1-k2)
        y = k1*x+c1
        return (int(x),int(y))


    def get_point_line_dist(self,point,k,c):
        '''
        计算点到直线的距离
        point：点
        k :斜率
        c：截据
        '''
        x0 = point[0]
        y0 = point[1]

        d = abs(k*x0-y0 + c)/(k**2 + 1)**0.5
        return d

    def get_dist_lines(self,dist,k,point):
        '''
        获取距离当前直线距离为x的两条直线的截据

        dist: 距离
        k：斜率
        point ： 在直线上的一点
        '''

        x1 = point[0]
        y1 = point[1]

        c1 = y1 - k * x1 -dist * (k**2+1)**0.5
        c2 = dist * (k**2+1)**0.5 - y1 + k*x1

        return (c1,c2)

    def yaw_left_or_right(self , current_pos ,current_pos_end ,point ):
        '''
            判断点在当前位置指向的左侧，右侧或者当前直线上
            
            current_pos : 当前位置坐标
            current_pos_end ：当前位置前进方向上的末位点
            point ： 需要判断关系的点

            return : 计算出的值 。d大于0 在左侧，小于0 在右侧，等于0在直线上


            设直线是由其上两点(x1,y1)，(x2,y2)确定的，直线方向是由(x1,y1)到(x2,y2)的方向。

            假设直线方程为：Ax+By+C=0，则有：

            　　　　A=y2-y1; 　　B=x1-x2;　　C=x2*y1-x1*y2;

            这时可以通过计算D，来判断点P是在直线的哪一侧：
            　　　　D=A*xp+B*yp+C

            若D<0，则点P在直线的左侧；若D>0，则点P在直线的右侧；若D＝0，则点P在直线上。
            d < 0 左侧逆时针取值，d > 0右侧顺时针，d=0中间顺时针
            左侧返回True  右侧和中间返回False
        '''
        self.current_pos_end = current_pos_end

        x1 = current_pos[0]
        y1 = current_pos[1]

        x2 = current_pos_end[0]
        y2 = current_pos_end[1]

        A = y2 -y1
        B = x1 -x2
        C = x2*y1 - x1*y2

        D = A*point[0]+B*point[1]+C

        if D<0:
            return True
        else :
            return False


    def move_aircraft(self,vec):
        while not self.command_queue.empty():
            sleep(self.sleeptime)
        self.command_queue.put({'type':'move','movement':vec})
        while self.returnval_queue.empty():
            sleep(self.sleeptime)
        result = self.returnval_queue.get()
        return result
    '''
    def drive(self): 
        relative_pos=(0,0,0)
        while self.current_pos!=self.end_pos:           # Till task is finished:
            self.algo = A_star(self.end_pos)
            print 'Move 1 step'
            obstacle_map=self.get_obstacles_around()  # TODO:加入障碍记忆.
            print 'From ',self.current_pos
            for item in obstacle_map:
                self.obstacle_set.add(item)
            import time
            t1 = time.time()
            path = self.algo.find_path(self.current_pos,list(self.obstacle_set))
            t2 = time.time()
            print 'A* time cost:' ,(t2-t1)
            if not path:
                return
            print 'path:',path
            for next_move in path:                      #Till path has run out to a invalid point or finished.
                print 'current_pos:',self.current_pos
                next_pos = next_move
                relative_pos = (next_pos[0]-self.current_pos[0],next_pos[1]-self.current_pos[1],next_pos[2]-self.current_pos[2])
                print 'next_move : ',next_move
                print "relativ_move : ",relative_pos
                if not self.algo.is_valid(next_pos,list(self.obstacle_set)):
                    break
                self.current_pos = next_pos
                print 'ob_set:',list(self.obstacle_set)
                
                self.move_aircraft(relative_pos)
                sleep(0.5)
                obstacle_map=self.get_obstacles_around()
                for item in obstacle_map:
                    self.obstacle_set.add(item)
                predict_move = (self.current_pos[0]+relative_pos[0],self.current_pos[1]+relative_pos[1],self.current_pos[2]+relative_pos[2])
                print 'ob_map : ',obstacle_map
                print "prdict_move : ",predict_move
                if not self.algo.is_valid(predict_move,list(self.obstacle_set)):
                    print 'cant goooooo'
                    break
'''


