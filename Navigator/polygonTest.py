from shapely.geometry import Point, Polygon,MultiPoint,MultiPolygon,LinearRing,LineString,JOIN_STYLE,CAP_STYLE
from shapely.ops import cascaded_union, unary_union,nearest_points

from shapely.affinity import rotate,scale
import matplotlib.pyplot as plt

from descartes import PolygonPatch
import math

# p = [(108.0, -46.0), (111.0, -48.0), (112.0, -46.0), (112.0, -47.0), (112.0, -48.0), (111.0, -49.0), (108.0, -48.0), (107.0, -48.0), (104.0, -48.0), (102.0, -47.0), (100.0, -47.0), (98.0, -47.0), (97.0, -46.0), (96.0, -45.0), (97.0, -45.0), (98.0, -45.0), (99.0, -45.0), (100.0, -45.0), (101.0, -45.0), (103.0, -45.0), (105.0, -46.0), (108.0, -46.0)]
# p1 =  [(108.0, -56.0), (111.0, -58.0), (112.0, -56.0), (112.0, -57.0), (112.0, -58.0), (111.0, -59.0), (108.0, -58.0), (107.0, -58.0), (104.0, -58.0), (102.0, -57.0), (100.0, -57.0), (98.0, -57.0), (97.0, -56.0), (96.0, -55.0), (97.0, -55.0), (98.0, -55.0), (99.0, -55.0), (100.0, -55.0), (101.0, -55.0), (103.0, -55.0), (105.0, -56.0), (108.0, -56.0)]

l1 = LineString([(0,0),(0,1),(-0.25,0.25),(2,0.25)])
# l2 = LineString([(-0.25,0.25),(2,0.25)])

# l3 = unary_union([l1,l2])
l4 = l1.simplify(0.8)
print l1
print l4

# p =  list(Point ((1,2)).coords)

# print p[0]
# ps = Polygon(p)
# ps1 = Polygon(p1)
# ms = MultiPolygon([ps,ps1])
# # print (s)
# s = ms.simplify(0.7,preserve_topology=False)

# n = nearest_points(Point(0,0),ps1)
# # for i in n :

# print (n[1].distance(Point(0,0)))
# print (ps1.distance(Point(0,0)))

# l1 = LineString ([(-2,0),(0,1)])
# l2 = rotate(l1, -90,origin= (0,0))

# l3 = scale(l1,10,10,origin=(-2,0))
# print (l1.length)
# print (l2.length)
# print (l3.length)
# s = MultiPolygon([s,s])
# cascaded_union
# print (ps)
# l = LineString([(-0.5,0.5),(0.5,0.5)])

# print (l)
# s = ps.intersection(l)


# polygon = [(4.0, -2.0), (5.0, -2.0), (4.0, -3.0), (3.0, -3.0), (4.0, -2.0)]
# shapely_poly = Polygon(polygon)

# polygon1 = [(14.0, -2.0), (15.0, -2.0), (14.0, -3.0), (13.0, -3.0), (14.0, -2.0)]
# shapely_poly1 = Polygon(polygon1)

# poly = MultiPolygon([shapely_poly,shapely_poly1])

# line = [(2, -2.5), (16, -2.5)]
# shapely_line = LineString(line)

# print (list(shapely_line.coords)[1])

# print shapely_poly.intersection(shapely_line)
# intersection_line = list(shapely_poly.intersection(shapely_line).coords)
# print intersection_line
# print (p.convex_hull)

# ls = l.parallel_offset(0,side="right", join_style=JOIN_STYLE.round)

# s = l.simplify(0.7, preserve_topology=True)



# s = l.boundary
# print (s)

# s = l.buffer(-0.3,resolution=1, cap_style=CAP_STYLE.square,join_style=JOIN_STYLE.bevel)
# ps= ls.convex_hull

# # print (ps)
# GM = (math.sqrt(5)-1.0)/2.0
# W = 8.0
# H = W*GM
# SIZE = (W, H)

# BLUE = '#6699cc'
# GRAY = '#ff0000'

# # polygons = Polygon (p)
# # Make new figure for the merged polygon
# fig2 = plt.figure(2, figsize=(10,10), dpi=90)
# ax2 = fig2.add_subplot(111)

# patch2b = PolygonPatch(l1.convex_hull,fc = BLUE , ec=BLUE, alpha=0.5, zorder=2)
# ax2.add_patch(patch2b)
# patch2b = PolygonPatch(l2.convex_hull, fc =GRAY , ec=GRAY, alpha=0.5, zorder=2)
# ax2.add_patch(patch2b)

# xrange = [90, 120]
# yrange = [-40, -60]
# ax2.set_xlim(*xrange)
# ax2.set_xticks(range(*xrange) + [xrange[-1]])
# ax2.set_ylim(*yrange)
# ax2.set_yticks(range(*yrange) + [yrange[-1]])
# ax2.set_aspect(1)


# plt.show()