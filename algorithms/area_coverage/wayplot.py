# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from scipy import interpolate

def radar_plot(data,bounding_box):
    points = np.vstack((data[:,0],data[:,1]))
    grid_x, grid_y = np.mgrid[bounding_box[0]:bounding_box[1]:500j, bounding_box[2]:bounding_box[3]:500j]
    f = interpolate.griddata(points.T, data[:,2], (grid_x, grid_y), method='linear')
    colors=["magenta","blueviolet","royalblue","aqua","springgreen","greenyellow","yellow","orangered","red","white"]
    clrmap=mcolors.LinearSegmentedColormap.from_list("mycmap",colors)
    plt.figure(figsize=(10,5))
    plt.pcolor(grid_x,grid_y,f,cmap=clrmap)
    clb=plt.colorbar()

def way_plot(xlist,ylist,data,bounding_box):
    points = np.vstack((data[:,0],data[:,1]))
    grid_x, grid_y = np.mgrid[bounding_box[0]:bounding_box[1]:100j, bounding_box[2]:bounding_box[3]:100j]
    f = interpolate.griddata(points.T, data[:,2], (grid_x, grid_y), method='linear')
    colors=["magenta","blueviolet","royalblue","aqua","springgreen","greenyellow","yellow","orangered","red","white"]
    clrmap=mcolors.LinearSegmentedColormap.from_list("mycmap",colors) 
    plt.ion                      # 打开一个图窗
    plt.clf()                    # 删除原来的图像
    plt.pcolor(grid_x,grid_y,f,cmap=clrmap)
    clb=plt.colorbar()
    plt.plot(xlist ,ylist,'ko')
    plt.pause(0.01)                # 暂停0.001秒




