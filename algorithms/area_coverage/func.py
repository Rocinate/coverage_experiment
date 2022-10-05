# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import numpy as np
from scipy.spatial.distance import pdist, squareform
np.random.seed(42)

class Func:
    def __init__(self, positionStart, positionEnd, angleStart, angleEnd, radius, vMax, cov, delta, epsilon):
        self.positionStart = positionStart
        self.positionEnd = positionEnd
        self.angleStart = angleStart
        self.angleEnd = angleEnd
        self.radius = radius
        self.vMax = vMax
        self.cov = cov
        self.delta = delta
        self.epsilon = epsilon

    # agentPos: 智能体位置信息，第一列x，第二列y
    # ueHisY: 历史控制量
    # agentAngles: 智能体朝向
    # angles: 扇面角度
    # angleStart: 限定角度
    # angleEnd: 限定角度
    # radius： 连通半径
    # radarPos: 雷达位置
    # vMax: 最大速度限制
    # cov: 角度覆盖范围，通常为2°
    def ccangle(self, agentPos, angles, ueHisY, agentAngles):
        # 智能体数目
        n = agentPos.shape[0]
        ue = np.zeros(n)
        bestAngle = np.zeros(n)

        # 限制智能体角度范围
        lessIndex = angles < self.angleStart
        largerIndex = angles > self.angleEnd
        bestAngle[lessIndex] = self.angleStart + self.cov / 2
        bestAngle[largerIndex] = self.angleEnd - self.cov / 2

        # 得到距离矩阵
        distMatrix = squareform(pdist(agentPos, 'euclidean'))

        # 未赋值，说明智能体角度在限制范围内
        for index, value in enumerate(bestAngle):
            if value == 0:
                # 获取邻居距离
                distConnected = distMatrix[index, :]
                connectedIndex = np.where(distConnected <= self.radius)[0]
                # print(angles)
                # 取出邻居朝向角
                angleList = [self.angleStart, self.angleEnd] + \
                    angles[connectedIndex].tolist()
                angleList = np.array(sorted(angleList))

                indexPos = np.where(angleList == angles[index])[0][0]
                bestAngle[index] = (
                    angleList[indexPos - 1] + angleList[indexPos + 1]) / 2
        # 影响程度受距离的影响，距离雷达越近，通常的影响越小
        beta = 0.1
        gamma = 0.01
        gradient = (gamma - beta) / (self.positionEnd - self.positionStart)
        intercept = beta - self.positionStart * gradient

        alpha = gradient * agentPos[:, 0] + intercept
        sameTrendIndex = ueHisY * (angles - bestAngle) >= 0

        ue[sameTrendIndex] = ueHisY[sameTrendIndex] + alpha[sameTrendIndex] * \
            (angles[sameTrendIndex] - bestAngle[sameTrendIndex])
        # 相反运动趋势
        ue[~sameTrendIndex] = -ueHisY[~sameTrendIndex] + alpha[~sameTrendIndex] * \
            (angles[~sameTrendIndex] - bestAngle[~sameTrendIndex])
        temp = np.abs(ue)
        # 取最小值
        temp[temp > self.vMax] = self.vMax
        ue = np.sign(ue) * temp
        # print(ue[:, 1])

        # 保证无人机相邻时刻转角的幅度在pi/12之内
        angleChangeIndex = np.abs(
            np.arctan(ue/self.vMax) - agentAngles) > np.pi / 30
        # 符合条件
        newAngle = agentAngles[angleChangeIndex] + np.sign(
            angles[angleChangeIndex] - bestAngle[angleChangeIndex]) * np.pi / 30
        ue[angleChangeIndex] = self.vMax * np.tan(newAngle)

        # 保证无人机速度范围在-pi/3 ~ pi/3
        lessIndex = np.arctan(ue / self.vMax) < -np.pi / 3
        largerIndex = np.arctan(ue / self.vMax) > np.pi / 3

        ue[lessIndex] = self.vMax * np.tan(-np.pi / 3)
        ue[largerIndex] = self.vMax * np.tan(np.pi / 3)
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

    

    # 计算组间连通信息
    def calcGroupConnectBatch(self, positions, targetPos, batchNum, damageIndex, returnY, u_t):
        pass
