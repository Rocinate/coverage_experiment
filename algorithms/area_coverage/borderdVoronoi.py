# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import numpy as np
import scipy.spatial as sp
import sys

eps = sys.float_info.epsilon

class Vor:
    # 初始化维诺Class
    def __init__(self, box):
        self.box = box

    # 判断点是否在场地范围内
    def in_box(self, towers):
        return np.logical_and(np.logical_and(self.box[0] <= towers[:, 0],
                                             towers[:, 0] <= self.box[1]),
                              np.logical_and(self.box[2] <= towers[:, 1],
                                             towers[:, 1] <= self.box[3]))

    # 利用对称性质，生成带有边界点的维诺划分
    def voronoi(self, towers):
        # 选取在范围内的点
        i = self.in_box(towers)
        # 对范围内的点按照边界进行镜像
        points_center = towers[i, :]
        points_left = np.copy(points_center)
        points_left[:, 0] = self.box[0] - \
            (points_left[:, 0] - self.box[0])
        points_right = np.copy(points_center)
        points_right[:, 0] = self.box[1] + \
            (self.box[1] - points_right[:, 0])
        points_down = np.copy(points_center)
        points_down[:, 1] = self.box[2] - \
            (points_down[:, 1] - self.box[2])
        points_up = np.copy(points_center)
        points_up[:, 1] = self.box[3] + (self.box[3] - points_up[:, 1])
        points = np.append(points_center,
                           np.append(np.append(points_left,
                                               points_right,
                                               axis=0),
                                     np.append(points_down,
                                               points_up,
                                               axis=0),
                                     axis=0),
                           axis=0)
        # 计算维诺划分
        vor = sp.Voronoi(points)
        # 过滤无限划分区域
        regions = []
        for region in vor.regions:
            flag = True
            for index in region:
                if index == -1:
                    flag = False
                    break
                else:
                    x = round(vor.vertices[index, 0], 2)
                    y = round(vor.vertices[index, 1], 2)
                    if not(self.box[0] - eps <= x and x <= self.box[1] + eps and
                           self.box[2] - eps <= y and y <= self.box[3] + eps):
                        flag = False
                        break
            if region != [] and flag:
                regions.append(region)
        vor.filtered_points = points_center
        vor.filtered_regions = regions
        return vor

    # 带权重维诺质心计算
    # field_strength场强， xy - 场强
    def centroid_region(self, field_strength):
        x = field_strength[:, 0]
        y = field_strength[:, 1]
        strength = field_strength[:, 2]
        Cx = sum(x*strength) / sum(strength)
        Cy = sum(y*strength) / sum(strength)
        return Cx, Cy