# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import numpy as np
from scipy.spatial.distance import pdist, squareform

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
def ccangle(agentPos, angles, ueHisY, agentAngles, angleStart, angleEnd, radius, vMax, cov):
    # 智能体数目
    n = agentPos.shape[0]
    ue = np.zeros((n, 2))
    bestAngle = np.zeros(n)

    # 限制智能体角度范围
    largerIndex = angles > angleEnd
    lessIndex = angles < angleStart
    bestAngle[lessIndex] = angleStart + cov / 2
    bestAngle[largerIndex] = angleEnd - cov / 2

    # 得到距离矩阵
    distMatrix = squareform(pdist(agentPos, 'euclidean'))

    # 未赋值，说明智能体角度在限制范围内
    for index, value in enumerate(bestAngle):
        if value == 0:
            # 获取邻居距离
            distConnected = distMatrix[index, :]
            connectedIndex = np.where(distConnected <= radius)[0]
            # print(angles)
            # 取出邻居朝向角
            angleList = [angleStart, angleEnd] + angles[connectedIndex].tolist()
            angleList = np.array(sorted(angleList))

            indexPos = np.where(angleList == angles[index])[0][0]
            bestAngle[index] = (angleList[indexPos - 1] + angleList[indexPos + 1]) / 2
    # 影响程度受距离的影响，距离雷达越近，通常的影响越小
    beta = 1000
    alpha = (0.05 - beta) * agentPos[:, 0] / 17100 + (beta * 181 -0.5)/171
    sameTrendIndex = ueHisY * (angles - bestAngle) >= 0

    ue[sameTrendIndex, 1] = ueHisY[sameTrendIndex] + alpha[sameTrendIndex] * (angles[sameTrendIndex] - bestAngle[sameTrendIndex])
    # 相反运动趋势
    ue[~sameTrendIndex, 1] = -ueHisY[~sameTrendIndex] + alpha[~sameTrendIndex] * (angles[~sameTrendIndex] - bestAngle[~sameTrendIndex])
    temp = np.abs(ue[:, 1])
    # 取最小值
    temp[temp > vMax] = vMax
    ue[:, 1] = np.sign(ue[:, 1]) * temp
    # print(ue[:, 1])

    # 保证无人机相邻时刻转角的幅度在pi/6之内
    angleChangeIndex = np.abs(np.arcsin(ue[:, 1]/vMax) - agentAngles) <= np.pi / 18
    # 符合条件
    ue[angleChangeIndex, 0] = np.sqrt(vMax**2 - ue[angleChangeIndex, 1] ** 2)
    # 不符合条件
    newAngle = agentAngles[~angleChangeIndex] + np.sign(angles[~angleChangeIndex] - bestAngle[~angleChangeIndex]) * np.pi / 18
    ue[~angleChangeIndex, 0] = vMax * np.cos(newAngle)
    ue[~angleChangeIndex, 1] = vMax * np.sin(newAngle)

    # 保证无人机速度范围在-pi/3 ~ pi/3
    lessIndex = np.arcsin(ue[:, 1] / vMax) < -np.pi / 3
    largerIndex = np.arcsin(ue[:, 1] / vMax) > np.pi / 3

    ue[lessIndex, 0] = vMax * np.cos(-np.pi / 3)
    ue[lessIndex, 1] = vMax * np.sin(-np.pi / 3)
    ue[largerIndex, 0] = vMax * np.cos(np.pi / 3)
    ue[largerIndex, 1] = vMax * np.sin(np.pi / 3)
    return ue
