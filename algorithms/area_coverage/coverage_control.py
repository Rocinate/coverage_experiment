# -*- coding: UTF-8 -*-
#!/usr/bin/env python

import numpy as np
import matplotlib.path as mpltPath
# 自定义函数
from VoronoiBounded import voronoi

#判断点(x,y)是否在以数组vertices的多边形的区域内，在区域内或者区域边界上返回的值为Ture
def inpolygon(x, y, vertices):
    path = mpltPath.Path(vertices)
    in_on = path.contains_points([[x,y]], radius=-1e-10)
    return in_on

# 计算有场强的维诺质心
def P_Centroid(upda):
    upda1=upda[:,0]
    upda2=upda[:,1]
    upda3=upda[:,2]
    Cx=(sum(upda1*upda3))/(sum(upda3))
    Cy=(sum(upda2*upda3))/(sum(upda3))
    return Cx,Cy

def con_control(position, bounding_box,A):
    # 维诺划分
    vor = voronoi(np.array(
            [position[key] for key in position]
        ), bounding_box)
    posList = np.array([position[key] for key in position])
    Idlist = list(position.keys())
    # 创建字典用来存储ue，但由于维诺划分不能保证区域的顺序与无人机的编号顺序相同,因此需要通过字典的key来匹配
    ue = {K:[0,0] for K in Idlist}

    # 遍历维诺区域
    for region in vor.filtered_regions:
        vertices = vor.vertices[region + [region[0]], :]
        path = mpltPath.Path(vertices)
        # 判断是哪一个无人机在当前的维诺划分区域内，并返回该无人机位置的索引值
        index = np.where(path.contains_points(posList))[0][0]
        key0 = Idlist[index]
        # 创建列表来存储满足条件的场强点
        cen = []
        # 遍历场强点
        for i in range(len(A)):
            Ax = A[i,0]
            Ay = A[i,1]
            # 判断当前的场强点是否在当前的维诺区域内
            if inpolygon(Ax,Ay,vertices):
                cen.append(A[i,:].tolist())
        upda = np.array(cen)
        # 计算当前区域的维诺质心
        Cx,Cy = P_Centroid(upda)
        Xrange = bounding_box[1]
        Yrange = bounding_box[3]
        bounding_vertices = np.array([[-Xrange,-Yrange],
                                      [-Xrange, Yrange],
                                      [ Xrange, Yrange],
                                      [ Xrange,-Yrange],
                                      [-Xrange,-Yrange]])
        if inpolygon(Cx,Cy,bounding_vertices):
            ue_x = Cx - position[key0][0]
            ue_y = Cy - position[key0][1]
            ue[key0] = np.array([ue_x,ue_y])

    return ue
