# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import matplotlib.pyplot as plt

class Graph:
    def __init__(self, IdList, xRange, yRange):
        # 获取命名空间
        names = self.__dict__

        # 创建绘图句柄
        fig, ax = plt.subplots()

        # 根据无人机ID动态申请变量
        for Id in IdList:
            # 加逗号，不然取不出来
            names['line' + Id], = plt.plot([], [], '.-', label='flight'+Id)
            names['track_his' + Id] = [[], []]
        # 仅显示✈的label
        plt.legend()
        for Id in IdList:
            names['ridges_handle' + Id], = plt.plot([], [], 'k-', label='ridges')

        self.centroids_handle, = plt.plot([], [], 'go', label='centriods')

        # 规定画图范围
        ax.set_xlim([-xRange, xRange])
        ax.set_ylim([-yRange, yRange])

    # 更新轨迹信息
    def updateTrack(self, waypoints, Id):
        # 根据ID动态执行
        exec("self.track_his{}[0].append(waypoints[:, 0].tolist())".format(Id))
        exec("self.track_his{}[1].append(waypoints[:, 1].tolist())".format(Id))
        exec("self.line{}.set_data(self.track_his{}[0], self.track_his{}[1])".format(
            Id, Id, Id))
        plt.pause(0.0001)

    # 更新维诺划分区域
    def updateRidges(self, positionWithId):
        for cf in positionWithId:
            ridges = cf['vertices']
            Id = str(cf['Id'])
            exec("self.ridges_handle{}.set_data(ridges[:, 0], ridges[:, 1])".format(Id))

    # 更新维诺质心
    def updateCentroid(self, centroid):
        self.centroids_handle.set_data(centroid[:, 0], centroid[:, 1])
