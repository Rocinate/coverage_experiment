# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import numpy as np
import scipy.spatial as sp
import sys
import matplotlib.path as mpltPath
import math
import copy

eps = sys.float_info.epsilon

class Vor:
    # 初始化维诺Class
    def __init__(self, box, lineSpeed, angularSpeed):
        self.box = box
        self.lineSpeed = lineSpeed
        self.angularSpeed = angularSpeed
    
    # 判断点是否在场地范围内
    def __in_box(self, towers, bounding_box):
        return np.logical_and(np.logical_and(bounding_box[0] <= towers[:, 0],
                                             towers[:, 0] <= bounding_box[1]),
                              np.logical_and(bounding_box[2] <= towers[:, 1],
                                             towers[:, 1] <= bounding_box[3]))

    # 利用对称性质，生成带有边界点的维诺划分
    def __voronoi(self, towers, bounding_box):
        # 选取在范围内的点
        i = self.__in_box(towers, bounding_box)
        # 对范围内的点按照边界进行镜像
        points_center = towers[i, :]
        points_left = np.copy(points_center)
        points_left[:, 0] = bounding_box[0] - \
            (points_left[:, 0] - bounding_box[0])
        points_right = np.copy(points_center)
        points_right[:, 0] = bounding_box[1] + \
            (bounding_box[1] - points_right[:, 0])
        points_down = np.copy(points_center)
        points_down[:, 1] = bounding_box[2] - \
            (points_down[:, 1] - bounding_box[2])
        points_up = np.copy(points_center)
        points_up[:, 1] = bounding_box[3] + (bounding_box[3] - points_up[:, 1])
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
                    if not(bounding_box[0] - eps <= x and x <= bounding_box[1] + eps and
                           bounding_box[2] - eps <= y and y <= bounding_box[3] + eps):
                        flag = False
                        break
            if region != [] and flag:
                regions.append(region)
        vor.filtered_regions = regions
        return vor

    # 维诺质心计算
    def __centroid_region(self, vertices):
        # Polygon's signed area
        A = 0
        # Centroid's x
        C_x = 0
        # Centroid's y
        C_y = 0
        for i in range(0, len(vertices) - 1):
            s = (vertices[i, 0] * vertices[i + 1, 1] -
                vertices[i + 1, 0] * vertices[i, 1])
            A = A + s
            C_x = C_x + (vertices[i, 0] + vertices[i + 1, 0]) * s
            C_y = C_y + (vertices[i, 1] + vertices[i + 1, 1]) * s
        A = 0.5 * A
        C_x = round((1.0 / (6.0 * A)) * C_x, 2)
        C_y = round((1.0 / (6.0 * A)) * C_y, 2)
        return [C_x, C_y]

    def updateVor(self, positionWithId):
        towers = np.array(
            [cf['Position'] for cf in positionWithId]
        )
        # 获取维诺划分
        vor = self.__voronoi(towers, self.box)
        # 获取无人机Id
        IdList = [cf['Id'] for cf in positionWithId]

        vorResult = []

        for region in vor.filtered_regions:
            # 获取顶点划分出来的范围
            vertices = vor.vertices[region + [region[0]], :]
            path = mpltPath.Path(vertices)

            # 计算质心
            centroid = self.__centroid_region(vertices)

            # 通过维诺顶点划分出来的范围寻找标号
            # 因为维诺划分出来的结果和输入进去的貌似顺序不同🤣
            Id = IdList[np.where(path.contains_points(towers))[0][0]]

            vorResult.append({
                'Id': Id,
                'vertices': vertices,
                'centroid': centroid
            })
        
        return vorResult
    
    def virtualVor(self, positionWithId):
        return self.updateVor(self.virtualPosition(positionWithId))

    def virtualPosition(self, positionWithId):
        # 深拷贝，避免更改到原址
        virtualList = copy.deepcopy(positionWithId)
        # 转换为虚拟位置
        for cf in virtualList:
            position = cf['Position']
            pose = cf['Pose']
            virtualPosition = [
                round(position[0] - (self.lineSpeed/self.angularSpeed) * (math.sin(pose)),2),
                round(position[1] - (self.lineSpeed/self.angularSpeed) * (math.cos(pose)),2)
            ]
            cf['Position'] = virtualPosition
        return virtualList
