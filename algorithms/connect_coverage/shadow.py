from turtle import position
import numpy as np
np.random.seed(42)

positionStart = -2.5
positionEnd = 3.0

class Shadow:
    def __init__(self, gutter, n, batchNum, damageIndex, vMax, vBack):
        self.n = n
        self.gutter = gutter
        # 多批次无人机参数
        self.batchNum = batchNum
        self.damageIndex = damageIndex
        self.vMax = vMax
        self.vBack = vBack
        self.positionStart = positionStart
        self.positionEnd = positionEnd

        self.radarPos = np.array([
            [6, 10],
            [6, 0],
            [6, -10]
        ])


        self.getBatchDiff()
        self.initBatchParams()

    # 初始化连通批次无人机参数
    def initBatchParams(self):
        self.total = 14
        batchPos = np.zeros((self.total * 2, 2))

        # 每批次无人机数量
        eachNum = int(self.total / self.batchNum)
        # 随机初始化
        for i in range(self.batchNum * 2):
            batchPos[i*eachNum: (i+1)*eachNum, :] = np.random.rand((eachNum, 2))
            batchPos[i*eachNum: (i+1)*eachNum, 0] -= 5
            if i < self.batchNum:
                batchPos[i*eachNum: (i+1)*eachNum, 1] += self.gutter
            else:
                batchPos[i*eachNum: (i+1)*eachNum, 1] -= self.gutter
        self.batchPos = batchPos
        self.u_t = np.array([1]*self.total + [0]*self.total)

    # 计算镜像区域位移
    def getBatchDiff(self):
        diff1 = np.random.rand(self.n, 2)
        diff1[:, 0] = diff1[:, 0] / 2
        diff2 = np.random.rand(self.n, 2)
        diff2[:, 0] = diff1[:, 0] / 2
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
            positions[flightIndex, 0] = -16/121 * positions[flightIndex, 0]**2 + \
                8/121 * positions[flightIndex, 0] + \
                1 + positions[flightIndex, 0]
            positions[flightIndex, 1] = positions[flightIndex, 1]

    def updateConnect(self):
        pass

    def calcGroupConnectBatch(self, uavPos, radarPos, batchNum, damageIndex, returnY, u_t):
        num = uavPos.shape[0]
        # 可用无人机数量
        uavNum = num - len(damageIndex)
        numPerBatch = round(uavNum / batchNum)
        minUavNum = np.array([numPerBatch] * (batchNum-1)+[uavNum - numPerBatch * (batchNum-1)])

        nextPos = np.zeros((self.total, 2))

        # 分配位置信息
        # 飞机数量索引，批次编号索引
        j, k, a = 0, 0, 0
        mid = []
        finishBatch = []
        isFinish = False

        for i in range(num):
            if i in damageIndex:
                continue

            #  非损坏飞机
            nextPos[i, :] = radarPos[0]+j*(radarPos[1] - radarPos[0])/ (minUavNum(k)+1)
            j += 1
            a += uavPos[i, 0]

            if uavPos[i, 0] >= self.positionEnd:
                isFinish = isFinish or True

            if j > minUavNum(k):
                mid.append(a/(j-1))
                finishBatch.append(isFinish)
                isFinish = False
                j, k, a = 0, k+1, 0

        # 降序
        batchIndex = np.argsort(mid, reversed=True)

        for i in range(batchNum):
            batchId = batchIndex[i]
            if i > 0:
                # 修改中线位置，保证相邻两个批次的中线之间存在距离
                mid[batchId] = np.min(mid[batchId], mid[batchIndex[i-1]] - (self.positionEnd - self.positionStart))

        u = np.zeros((num, 2))
        connIndex = np.zeros(num)
        j, k = 0, 0
        for i in range(num):
            if i in damageIndex:
                continue
            



    def getGraphData(self, positions):
        result = np.zeros((self.n*3, 2))
        result[:self.n, :] = positions.copy()
        self.updateShadow(result)

        self.updateConnect(result)
        return result