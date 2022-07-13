import numpy as np
from scipy.spatial.distance import pdist, squareform

class Controller:
    def __init__(self, n, angleStart, angleEnd, radius, vMax, cov, delta, epsilon):
        self.n = n # 无人机数目
        self.angleStart = angleStart # 限定角度
        self.angleEnd = angleEnd # 限定角度
        self.radius = radius # 连通半径
        self.vMax = vMax # 最大速度限制
        self.cov = cov # cov 角度覆盖范围，通常为2°

        # 参数
        self.delta = delta
        self.epsilon = epsilon
    
    # 角度覆盖控制
    def ccangle(self, positions, angles, agentAngles):
        ue = np.zeros((self.n, 2))
        bestAngle = np.zeros(self.n)

        # 限制智能体角度范围
        largerIndex = angles > self.angleEnd
        lessIndex = angles < self.angleStart
        bestAngle[lessIndex] = self.angleStart + self.cov / 2
        bestAngle[largerIndex] = self.angleEnd - self.cov / 2

        # 得到距离矩阵
        distMatrix = squareform(pdist(positions, 'euclidean'))

        # 未赋值，说明智能体角度在限制范围内
        for index, value in enumerate(bestAngle):
            if value == 0:
                # 获取邻居距离
                distConnected = distMatrix[index, :]
                connectedIndex = np.where(distConnected <= self.radius)[0]
                # print(angles)
                # 取出邻居朝向角
                angleList = [self.angleStart, self.angleEnd] + angles[connectedIndex].tolist()
                angleList = np.array(sorted(angleList))

                indexPos = np.where(angleList == angles[index])[0][0]
                bestAngle[index] = (angleList[indexPos - 1] + angleList[indexPos + 1]) / 2
        # 影响程度受距离的影响，距离雷达越近，通常的影响越小
        beta = 0.1
        gamma = 0.01
        gradient = (gamma - beta) / 4.5
        intercept = (4 * gamma + 5 * beta) / 9
        # alpha = (0.05 - beta) * agentPos[:, 0] / 17100 + (beta * 181 -0.5)/171
        alpha = gradient * self.agentPos[:, 0] + intercept
        sameTrendIndex = self.ueHisY * (angles - bestAngle) >= 0

        ue[sameTrendIndex, 1] = self.ueHisY[sameTrendIndex] + alpha[sameTrendIndex] * (angles[sameTrendIndex] - bestAngle[sameTrendIndex])
        # 相反运动趋势
        ue[~sameTrendIndex, 1] = -self.ueHisY[~sameTrendIndex] + alpha[~sameTrendIndex] * (angles[~sameTrendIndex] - bestAngle[~sameTrendIndex])
        temp = np.abs(ue[:, 1])
        # 取最小值
        temp[temp > self.vMax] = self.vMax
        ue[:, 1] = np.sign(ue[:, 1]) * temp
        # print(ue[:, 1])

        # 保证无人机相邻时刻转角的幅度在pi/6之内
        angleChangeIndex = np.abs(np.arcsin(ue[:, 1]/self.vMax) - agentAngles) <= np.pi / 18
        # 符合条件
        ue[angleChangeIndex, 0] = np.sqrt(self.vMax**2 - ue[angleChangeIndex, 1] ** 2)
        # 不符合条件
        newAngle = agentAngles[~angleChangeIndex] + np.sign(angles[~angleChangeIndex] - bestAngle[~angleChangeIndex]) * np.pi / 18
        ue[~angleChangeIndex, 0] = self.vMax * np.cos(newAngle)
        ue[~angleChangeIndex, 1] = self.vMax * np.sin(newAngle)

        # 保证无人机速度范围在-pi/3 ~ pi/3
        lessIndex = np.arcsin(ue[:, 1] / self.vMax) < -np.pi / 2.5
        largerIndex = np.arcsin(ue[:, 1] / self.vMax) > np.pi / 2.5

        ue[lessIndex, 0] = self.vMax * np.cos(-np.pi / 2.5)
        ue[lessIndex, 1] = self.vMax * np.sin(-np.pi / 2.5)
        ue[largerIndex, 0] = self.vMax * np.cos(np.pi / 2.5)
        ue[largerIndex, 1] = self.vMax * np.sin(np.pi / 2.5)
        return ue

    # 联通保持控制
    def con_pre(self, positions, features, featureVec, distMatrix, A):
        '''
        #此控制律保持连通度不低于设定阈值。
        #features表示nx1特征值估计向量，featureVec表示对应的特征向量估计nx1。
        #x表示位置矩阵
        #d表示距离平方矩阵nxn，A表示邻接矩阵，R表示通信半径，deLta表示通信半径下的权值
        #epsilon表示连通度最小阈值。
        '''
        value = -(self.radius**2/np.log(self.delta))/2
        uc = np.zeros((self.n, 2))
        for i in range(self.n):
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
                a2x += (-(A[i, agent])/value)*(positions[i, 0] - positions[agent, 0])*((featureVec[i]-featureVec[agent])**2)
                a2y += (-(A[i, agent])/value)*(positions[i, 1] - positions[agent, 1])*((featureVec[i]-featureVec[agent])**2)

            ucx = -a1*a2x
            ucy = -a1*a2y

            uc[i, :] = np.array([ucx, ucy])
        return uc

    # 总控制率
    def get_control_rate(self, ):
        ue = self.ccangle()
        uc = self.con_pre()
        u = 0.1 * uc + ue

        # 限幅，避免出现超出arcsin范围的情况，导致返回nan
        for agent in range(self.n):
            dist = np.linalg.norm(u[agent, :])
            if dist > self.vMax:
                u[agent, :] = self.vMax * u[agent, :] / dist

        return u