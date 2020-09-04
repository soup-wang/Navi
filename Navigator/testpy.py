#encoding=utf-8

import sys
from descartes import PolygonPatch
import matplotlib.pyplot as plt
import alphashape
import random
import time

# import Polygon
points = [(22, -12), (23, -1), (21, -9), (24, 20), (21, -5), (22, -5), (28, 9), (19, 4), (18, 4), (25, 12), (22, -18), (22, 19), (25, -19), (20, 7), (19, -7), (21, 8), (16, -10), (22, 6), (21, 6), (18, -6), (23, 7), (28, -14), (23, -16), (28, -21), (25, -10), (22, -15), (29, 19), (19, 14), (21, -1), (16, 9), (23, -18), (20, 9), (19, 3), (18, 9), (25, 15), (24, -10), (21, 12), (18, -1), (19, -12), (21, 5), (22, 3), (29, 17), (17, -14), (29, -14), (16, 15), (28, 10), (20, -2), (29, -20), (22, -14), (18, 12), (28, 13), (19, 13), (28, -19), (17, 13), (24, 11), (23, -19), (21, 18), (20, 14), (19, -2), (18, 10), (21, -18), (23, -2), (24, -13), (21, 15), (28, 17), (19, -13), (24, 17), (17, -11), (29, -23), (22, 12), (29, 20), (20, -1), (18, -3), (21, 14), (23, 9), (28, 15), (20, -14), (29, 14), (29, -17), (16, 13), (21, -15), (20, 16), (19, 8), (18, 0), (17, 8), (24, 8), (17, 15), (25, -15), (20, 3), (19, -3), (20, -11), (29, -19), (24, -20), (21, 2), (28, 22), (16, -9), (17, -6), (22, 9), (29, 23), (22, -4), (25, 10), (28, 12), (22, -11), (24, -14), (20, -3), (21, -6), (16, 10), (17, -10), (19, 7), (18, 5), (17, 11), (22, -17), (22, 16), (17, -12), (20, 0), (19, -8), (18, 16), (28, -12), (22, 7), (17, -7), (21, 10), (23, 6), (22, 10), (28, -13), (18, -4), (18, -2), (20, -16), (24, 14), (22, -10), (23, -4), (19, 17), (28, -23), (21, -3), (24, 15), (20, 10), (19, 2), (18, 6), (25, 14), (24, 2), (21, 11), (20, 5), (19, -9), (23, 5), (22, 0), (29, 16), (28, 24), (18, 13), (29, -15), (22, -8), (16, -11), (28, 11), (28, -16), (24, 21), (29, 10), (21, 9), (29, -21), (22, -13), (14, 12), (20, -4), (19, 12), (28, -18), (17, 12), (24, 12), (25, -11), (20, 15), (19, 1), (18, 11), (25, 9), (24, -12), (21, -10), (28, 18), (19, -14), (23, -3), (22, 13), (20, -6), (29, -12), (25, -17), (23, 8), (25, 16), (22, -2), (15, 13), (29, 13), (20, -10), (29, -18), (16, 14), (19, 11), (20, -5), (21, -13), (15, 12), (21, -2), (25, -16), (20, 12), (19, -4), (18, -7), (15, 11), (24, -15), (21, 1), (28, 23), (19, -15), (23, 2), (22, 14), (29, 22), (18, 15), (24, -19), (17, 7), (16, 12), (21, -7), (16, 11), (24, -16), (20, -13), (19, 6), (18, 2), (17, 10), (29, -22), (22, 17), (25, -13), (20, 1), (19, -5), (25, 20), (24, -18), (17, -8), (28, 20), (18, -10), (23, 1), (22, 11), (20, -7), (25, 17), (18, 1), (20, 4), (22, -9), (19, 16), (28, -22), (21, -4), (22, -6), (25, 19), (28, -17), (20, 11), (19, 5), (18, 7), (25, 13), (17, -13), (22, 18), (25, -18), (20, 6), (19, -10), (18, -8), (22, 1), (21, 7), (24, 19), (29, -16), (23, -17), (23, 4), (20, -12), (28, -15), (21, 16), (21, -12), (29, 15), (21, -11), (22, -7), (22, -16), (19, 15), (24, 16), (24, 13), (22, 5), (25, -12), (20, 8), (19, 0), (18, 8), (25, 8), (24, -11), (21, 13), (28, 19), (19, -11), (22, 2), (29, 18), (25, 11), (25, 21), (29, -13), (20, -8), (19, -1), (24, 9), (25, 18), (29, 12), (20, -15), (22, -3), (20, 13), (19, 10), (28, -20), (21, -14), (24, 10), (25, -20), (18, -9), (20, -9), (21, -17), (21, -16), (21, 0), (28, 16), (19, -16), (18, -5), (22, 15), (29, 21), (17, -9), (28, 14), (21, 17), (23, -5), (29, 11), (24, 18), (18, -11), (21, -8), (22, -1), (15, 14), (19, 9), (18, 3), (17, 9), (24, 7), (22, 4), (25, -14), (20, 2), (19, -6), (18, 14), (21, 4), (24, -17), (21, 3), (28, 21), (17, 14), (23, 0), (22, 8), (29, 24), (21, 19)]
# for i in range(1,20) :
    # points.append((random.random(),random.random()))
# points = [(1,3),(1,5),(6,3)]
start_time = time.time()

#    x = np.array(list_forword)
#         xx = pd.DataFrame({"k": x})
#         yy = pd.Series(list_queue) 
#         res = pd.ols(y=yy, x=xx)   

'''
它的原理可以想象成一个半径为α 的圆在点集 s 外滚动，当α足够大时，
这个圆就不会滚到点集内部，其滚动的痕迹就是这个点集的边界线。因此，当α
值很小，则每个点都是边界；如果α 很大（α →∞ ）时，则求出的边界线为
点集 s 的凸包。
'''


alpha_shape = alphashape.alphashape(points, alpha=0.1)
end_time = time.time()

print (end_time - start_time)

octomap_alpha_shape = alpha_shape.__geo_interface__["coordinates"]
octomap_list = []
if len(octomap_alpha_shape) > 1 :
    for item in octomap_alpha_shape :
        octomap_list.append(item[0])
else :
    octomap_list = octomap_alpha_shape
    
current_pos = (-1,-1,15)
boundary_points = octomap_list[0]
nearest_point = boundary_points[0]

nearest_dist = ((boundary_points[0][0]-current_pos[0])**2 + (boundary_points[0][1]-current_pos[1])**2)**0.5
sign = True
for item in boundary_points:
    if sign : 
        sign = False
        continue
    dist = ((item[0]-current_pos[0])**2 + (item[1]-current_pos[1])**2)**0.5
    if dist < nearest_dist :
        nearest_dist = dist
        nearest_point = item
    

print (octomap_list[0])
print (nearest_dist)
# print (alpha_shape.__geo_interface__["coordinates"][0])
# print (octomap_list)
# point_num = len(ps)
# # if(point_num < 3): return 0.0
# s = ps[0][1] * (ps[point_num-1][0] - ps[1][0])
# #for i in range(point_num): # (int i = 1 i < point_num ++i):
# for i in range(1, point_num): # 有小伙伴发现一个bug，这里做了修改，但是没有测试，需要使用的亲请测试下，以免结果不正确。
#     s += ps[i][1] * (ps[i-1][0] - ps[(i+1)%point_num][0])
# area = abs(s/2.0)

# print (area)
# print (type(alpha_shape))
# print (alpha_shape.__geo_interface__["coordinates"][0])

# # alpha_shape._get_coords()
# # x,y=alpha_shape.xy()
# # alpha_shape.__geo_interface__.coordinates
# print (end_time-start_time)
# print (alpha_shape.__geo_interface__["coordinates"])

fig, ax = plt.subplots()
ax.scatter(*zip(*points))
ax.add_patch(PolygonPatch(alpha_shape, alpha=0.2))
plt.show()
