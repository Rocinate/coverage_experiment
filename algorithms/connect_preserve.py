# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import numpy as np

# 定义计算连通保持控制律函数
def con_pre(features, featureVec, positions, distMatrix, A, radius, delta, epsilon):
    '''
    #此控制律保持连通度不低于设定阈值。
    #features表示nx1特征值估计向量，featureVec表示对应的特征向量估计nx1。
    #x表示位置矩阵
    #d表示距离平方矩阵nxn，A表示邻接矩阵，R表示通信半径，deLta表示通信半径下的权值
    #epsilon表示连通度最小阈值。
    '''
    n = positions.shape[0]
    value = -(radius**2/np.log(delta))/2
    uc = np.zeros((n, 2))
    for i in range(n):
        if features[i] <= epsilon:
            a1 = -50000
        else:
            a1 = -10/(np.sinh(features[i] - epsilon) ** 2)

        a2x, a2y = 0, 0
        # 将距离矩阵取出，并删除自己
        distConnected = distMatrix[i, :]
        connectedIndex = np.where(distConnected <= radius)[0]
        connectedIndex = np.delete(connectedIndex, connectedIndex == i)

        for agent in connectedIndex:
            a2x += (-(A[i, agent])/value)*(positions[i, 0] - positions[agent, 0])*((featureVec[i]-featureVec[agent])**2)
            a2y += (-(A[i, agent])/value)*(positions[i, 1] - positions[agent, 1])*((featureVec[i]-featureVec[agent])**2)

        ucx = -a1*a2x
        ucy = -a1*a2y

        uc[i, :] = np.array([ucx, ucy])
    return uc
