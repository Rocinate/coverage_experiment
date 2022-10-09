# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import numpy as np
np.random.seed(42)

positionStart = -2.5
positionEnd = 3.0

class Shadow:
    def __init__(self, dt, gutter, n, vMax=0.5, vBack=0.6, batchNum=2, damageIndex=[[], []]):
        self.n = n
        self.gutter = gutter
        # 多批次无人机参数
        self.batchNum = batchNum
        self.damageIndex = damageIndex # [[1, 2, 3], [4, 5, 6]]
        self.vMax = vMax
        self.vBack = vBack
        self.positionStart = positionStart
        self.positionEnd = positionEnd
        self.dt = dt

        self.radarPos = np.array([
            [6, 8],
            [6, 0],
            [6, -8]
        ])

        self.returnY = np.array([[
            6, 4, 6, 4
        ], [
            -4, -6, -4, -6
        ]])


        self.getBatchDiff()
        self.initBatchParams()

    # 初始化连通批次无人机参数
    def initBatchParams(self):
        self.total = 12
        batchPos = np.zeros((2, self.total, 2))

        # 随机初始化
        batchPos = np.random.rand(2, self.total, 2)
        batchPos[:, :, 0] -= 5

        batchPos[0, :, 1] += self.gutter / 2
        batchPos[1, :, 1] -= self.gutter / 2
        self.batchPos = batchPos
        self.u_t = np.zeros((2, self.total, 2))
        self.u_t[:, :, 0] = self.vMax

    # 计算镜像区域位移
    def getBatchDiff(self):
        diff1 = np.random.rand(self.n, 2)
        diff1[:, 0] = diff1[:, 0] / 2
        diff2 = np.random.rand(self.n, 2)
        diff2[:, 0] = diff2[:, 0] / 2
        self.diff1, self.diff2 = diff1, diff2

    # 计算影子信息
    def updateShadow(self, positions):
        n = self.n
        positions[n:2*n, :] = positions[0:n, :] + self.diff1
        positions[n:2*n, 1] = positions[n:2*n, 1] + self.gutter

        positions[2*n:3*n, :] = positions[0:n, :] + self.diff2
        positions[2*n:3*n, 1] = positions[2*n:3*n, 1] - self.gutter

        for flightIndex in range(n, 2*n):
            positions[flightIndex, 0] = positions[flightIndex, 0]
            positions[flightIndex, 1] = self.gutter*2 - positions[flightIndex, 1]

        for flightIndex in range(2*n, 3*n):
            positions[flightIndex, 0] = -16.0/121 * positions[flightIndex, 0]**2 + \
                8.0/121 * positions[flightIndex, 0] + \
                1.0 + positions[flightIndex, 0]
            positions[flightIndex, 1] = positions[flightIndex, 1]

    def updateConnect(self):
        # 生成上半部分
        u1, _ = self.calcGroupConnectBatch(self.batchPos[0, :, :], self.radarPos[0:2, :], self.batchNum, self.damageIndex[0], self.returnY[0], self.u_t[0, :, :])
        self.batchPos[0, :, :] += u1 * self.dt
        self.u_t[0, :, :] = u1
        # 生成下半部分
        u2, _ = self.calcGroupConnectBatch(self.batchPos[1, :, :], self.radarPos[1:3, :], self.batchNum, self.damageIndex[1], self.returnY[1], self.u_t[1, :, :])
        self.batchPos[1, :, :] += u2 * self.dt
        self.u_t[1, :, :] = u2

    def calcGroupConnectBatch(self, uavPos, radarPos, batchNum, damageIndex, returnY, u_t):
        num = uavPos.shape[0]
        # 可用无人机数量
        uavNum = num - len(damageIndex)
        numPerBatch = round(uavNum / batchNum)
        minUavNum = np.array([numPerBatch] * (batchNum-1)+[uavNum - numPerBatch * (batchNum-1)])

        nextPos = np.zeros((self.total, 2))

        # 分配位置信息
        # 飞机数量索引，批次编号索引
        j, k, a = 1, 0, 0
        mid = []
        finishBatch = []
        isFinish = False

        for i in range(num):
            if i in damageIndex:
                continue

            #  非损坏飞机
            nextPos[i, :] = radarPos[0]+j*(radarPos[1] - radarPos[0])/ (minUavNum[k]+1)
            j += 1
            a += uavPos[i, 0]

            if uavPos[i, 0] >= self.positionEnd:
                isFinish = isFinish or True

            if j > minUavNum[k]:
                mid.append(a/(j-1))
                finishBatch.append(isFinish)
                isFinish = False
                j, k, a = 1, k+1, 0

        # 降序
        batchIndex = np.argsort(mid)[::-1]

        for i in range(batchNum):
            batchId = batchIndex[i]
            if i > 0:
                # 修改中线位置，保证相邻两个批次的中线之间存在距离
                mid[batchId] = np.min([mid[batchId], mid[batchIndex[i-1]] - (self.positionEnd - self.positionStart)])

        u = np.zeros((num, 2))
        connIndex = np.zeros(num)
        j, k = 1, 0
        for i in range(num):
            if not i in damageIndex:
                # 抵近无人机到达覆盖结束位置，执行返航
                if uavPos[i, 0] >= self.positionEnd and np.abs(u_t[i, 1]) != self.vBack:
                    if uavPos[i, 1] > (returnY[0] - returnY[1]) / 2:
                        u[i, :] = np.array([0, self.vBack])
                    else:
                        u[i, :] = np.array([0, -self.vBack])
                elif uavPos[i, 0] >= self.positionEnd and np.abs(u_t[i, 1]) == self.vBack:
                    u[i, :] = u_t[i, :]
                    if uavPos[i, 1] >= returnY[0] or uavPos[i, 1] <= returnY[1]:
                        u[i, :] = np.array([-self.vBack, 0])
                elif uavPos[i, 0] > self.positionStart and u_t[i, 0] == -self.vBack:
                    u[i, :] = u_t[i, :]
                elif uavPos[i, 1] <= self.positionStart and u_t[i, 0] == -self.vBack:
                    if uavPos[i, 1] >= returnY[0]:
                        u[i, :] = np.array([0, -self.vBack])
                    else:
                        u[i, :] = np.array([0, self.vBack])
                elif uavPos[i, 0] <= self.positionStart and u_t[i, 1] == -self.vBack or u_t[i, 1] == self.vBack:
                    u[i, :] = u_t[i, :]
                    if uavPos[i, 1] <= returnY[2] and uavPos[i, 1] >= returnY[3]:
                        u[i, :] = np.array([self.vMax, 0])
                else:
                    u[i, :] = np.array([self.vMax, nextPos[i, 1] - uavPos[i, 1]])

                    if finishBatch[k] == 0:
                        connIndex[i] = 1
                    speed = np.linalg.norm(u[i, :])
                    if speed > self.vMax:
                        u[i, :] = self.vMax * u[i, :] / speed
                    disturbance = 0
                    if uavPos[i, 0] >= self.positionEnd - 1:
                        disturbance = 0.1
                    else:
                        disturbance = 0.2

                    if uavPos[i, 0] > mid[k] + disturbance:
                        u[i, 0] = 0.3 * self.vMax
                    else:
                        u[i, 0] = self.vMax
                if j < minUavNum[k]:
                    j += 1
                else:
                    j = 1
                    k += 1
            else:
                u[i, :] = np.array([0, 0])
        return u, connIndex


    def getGraphData(self, positions):
        result = np.zeros((self.n*3, 2))
        result[:self.n, :] = positions.copy()
        self.updateShadow(result)
        self.updateConnect()
        result = np.vstack((result, self.batchPos[0, :, :], self.batchPos[1, :, :]))
        return result