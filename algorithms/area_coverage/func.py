# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import numpy as np
import matplotlib.path as mpltPath
from scipy.spatial.distance import pdist, squareform
from algorithms.area_coverage.borderdVoronoi import Vor
np.random.seed(42)

class Func:
    def __init__(self, radius, vMax, delta, epsilon, box, field_strength):
        self.radius = radius
        self.vMax = vMax
        self.delta = delta
        self.epsilon = epsilon
        self.box = box
        self.vor = Vor(box)
        self.field_strength = field_strength

    # 覆盖控制
    def con_control(self, position):
        # 维诺划分
        vor = self.vor.voronoi(position)
        # 创建字典用来存储ue，但由于维诺划分不能保证区域的顺序与无人机的编号顺序相同,因此需要通过字典的key来匹配
        ue = np.zeros(position.shape)

        # 遍历维诺区域
        for region in vor.filtered_regions:
            vertices = vor.vertices[region + [region[0]], :]
            path = mpltPath.Path(vertices)

            # 判断是哪一个无人机在当前的维诺划分区域内，并返回该无人机位置的索引值
            flightIndex = np.where(path.contains_points(position))[0][0]

            # 维诺范围内场强点
            index = np.where(path.contains_points(self.field_strength[:, :2]))[0]
            cover_field = self.field_strength[index, :]

            # 计算当前区域的维诺质心
            Cx, Cy = self.vor.centroid_region(cover_field)

            # 计算质心是否在当前划分范围内部
            ue_x = Cx - position[flightIndex][0]
            ue_y = Cy - position[flightIndex][1]
            ue[flightIndex] = np.array([ue_x, ue_y])

        return ue

    # 定义计算拉普拉斯矩阵的函数
    # 输入：x为N*n的状态矩阵，N为智能体数量，n为状态维度；R为通信范围
    # 输出：n*n的拉普拉斯矩阵L，邻接矩阵A，距离矩阵d
    def L_Mat(self, X):
        value = -(self.radius**2/np.log(self.delta))
        d = squareform(pdist(X, 'euclidean'))
        N = X.shape[0]
        A = np.zeros((N, N))
        for i in range(N):
            for j in range(i):
                if(d[i, j] <= self.radius):
                    A[i, j] = np.exp(-d[i, j]**2/value)
                else:
                    A[i, j] = 0
                A[j, i] = A[i, j]

        D = np.diag(np.sum(A, axis=1))
        L = D-A

        return L, A, d

    # 定义计算连通保持控制律函数
    def con_pre(self, features, featureVec, positions, distMatrix, A):
        '''
        #此控制律保持连通度不低于设定阈值。
        #features表示nx1特征值估计向量，featureVec表示对应的特征向量估计nx1。
        #x表示位置矩阵
        #d表示距离平方矩阵nxn，A表示邻接矩阵，R表示通信半径，deLta表示通信半径下的权值
        #epsilon表示连通度最小阈值。
        '''
        n = positions.shape[0]
        value = -(self.radius**2/np.log(self.delta))/2
        uc = np.zeros((n, 2))
        for i in range(n):
            if features[i] <= self.epsilon:
                a1 = -50000
            else:
                a1 = -10/(np.sinh(features[i] - self.epsilon) ** 2)

            a2x, a2y = 0, 0
            # 将距离矩阵取出，并删除自己
            distConnected = distMatrix[i, :]
            connectedIndex = np.where(distConnected <= self.radius)[0]
            connectedIndex = np.delete(connectedIndex, connectedIndex == i)

            for agent in connectedIndex:
                a2x += (-(A[i, agent])/value)*(positions[i, 0] -
                                               positions[agent, 0])*((featureVec[i]-featureVec[agent])**2)
                a2y += (-(A[i, agent])/value)*(positions[i, 1] -
                                               positions[agent, 1])*((featureVec[i]-featureVec[agent])**2)

            ucx = -a1*a2x
            ucy = -a1*a2y

            uc[i, :] = np.array([ucx, ucy])
        return uc

    # 判断机器人是否为关键机器人，根据距离划分，距离小于threshold的为近邻，大于threshold的为远邻；
    # 如果i的远邻为空，i非关键机器人；如果i所有远邻的邻居中，均存在至少一个为i的近邻，则不为关键机器人
    # 输出：flag为nx1向量，元素表示：1为关键，0为非关键
    # 输入：agent：机器人编号，d：距离矩阵
    def is_Critical_robot(self, d, factor):
        n = d.shape[0]
        flag = np.ones(n)
        threshold = self.radius * factor

        for index in range(n):
            a = 0
            # 远邻索引
            farNeighbor = np.bitwise_and(d[index, :] > threshold, d[index, :] <= self.radius)
            if len(d[index, farNeighbor]) == 0:
                flag[index] = 0
            else:
                # 远邻
                ind_if = np.where(farNeighbor)[0]
                # 近邻
                ind_in = np.where(d[index, :] <= threshold)[0]

                for ifIndex in ind_if:
                    for inIndex in ind_in:
                        ind_ck = np.where(d[inIndex, :] <= threshold)[0]

                        if ifIndex in ind_ck:
                            a += 1
                            break

                if a == len(ind_if):
                    flag[index] = 0

        return flag



