# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt

with open('log/3.log', 'r') as f:
    X, Y , Z , W = zip(*[[float(s) for s in line.split("---")] for line in f])


fig = plt.figure(num=2, figsize=(15, 8),dpi=80)
#使用add_subplot在窗口加子图，其本质就是添加坐标系
#三个参数分别为：行数，列数，本子图是所有子图中的第几个，最后一个参数设置错了子图可能发生重叠
ax1 = fig.add_subplot(2,2,1)  
ax2 = fig.add_subplot(2,2,2)
ax3 = fig.add_subplot(2,2,3)
ax1.plot(X, Y )
ax2.plot(X, Z )
ax3.plot(X, W )
plt.show()
