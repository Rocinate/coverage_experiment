# -*- coding: UTF-8 -*-
#!/usr/bin/env python
from multiprocessing import Process
import numpy as np
import traceback # 错误堆栈

# 自定义库
from algorithms.area_coverage.func import Func

# 参数设置
R = 25  # 通信半径
delta = 0.1  # 通信边界边权大小，越小效果越好
epsilon = 0.1  # 最小代数连通度
vMax = 0.1  # 连通保持最大速度（用于限幅）
warnEpsilon = 0.6 # 连通度警戒值

class Workers(Process):
    def __init__(self, name, res, allCrazyFlies, dt, epochNum, field_strength, box, realFlightNum):
        Process.__init__(self)
        self.res = res
        self.name = name
        self.epoch = 0
        self.dt = dt
        self.epochNum = epochNum
        self.getParams(allCrazyFlies)
        self.func = Func(R, vMax, delta, epsilon, box, field_strength)
        self.warnEpsilon = warnEpsilon
        self.realFlightNum = realFlightNum

    # 从配置文件中解析无人机相关参数
    def getParams(self, allCrazyFlies):
        self.IdList = [item['Id'] for item in allCrazyFlies]

        self.n = len(self.IdList) # 无人机数量

        # 转换为numpy数组
        self.positions = np.array([item['Position'] for item in allCrazyFlies])

    # 更新损失和连通度
    def updateLossConn(self):
        L, A, d = self.func.L_Mat(self.positions)

        value, vectors = np.linalg.eig(L)
        # 从小到大对特征值进行排序
        index = np.argsort(value)
        self.vectors = vectors[:, index]
        self.value = value[index]

        self.L = L
        self.A = A
        self.d = d

        print("时刻" + str(self.epoch)+ " 的连通度为" + str(self.value[1]))

    def inControl(self):
        epoch = self.epoch

        self.updateLossConn()
        # 连通控制量
        ue = self.func.con_control(self.positions)
        for index in range(ue.shape[0]):
            dist = np.linalg.norm(ue[index, :])
            if dist > vMax:
                ue[index, :] = vMax * ue[index, :] / dist

        features = np.ones(self.n) * self.value[1]
        featureVec = self.vectors[:, 1]
        # 覆盖控制量
        uc = self.func.con_pre(features, featureVec, self.positions, self.d, self.A)

        # 限幅
        for index in range(uc.shape[0]):
            dist = np.linalg.norm(uc[index, :])
            if dist > vMax:
                uc[index, :] = vMax * uc[index, :] / dist

        u = np.zeros(uc.shape)

        critical = self.func.is_Critical_robot(self.d, 0.7)

        for flightIndex in range(u.shape[0]):
            if self.value[1] <= self.warnEpsilon:
                if critical[flightIndex]:
                    u[flightIndex] = ue[flightIndex] + uc[flightIndex]
                else:
                    u[flightIndex] = ue[flightIndex] + 0.8 * uc[flightIndex]
            else:
                u[flightIndex] = ue[flightIndex]

        self.positions += u * self.dt

        # 发布对应无人机执行情况
        for k in range(self.realFlightNum):
            Px, Py = self.positions[k, :]
            self.res.put({
                "Px": Px,
                "Py": Py,
                "Id": self.IdList[k],
                "index": epoch,
                "ux": u[k, 0],
                "uy": u[k, 1],
                "uz": 0
            })

    def run(self):
        print("start calculating!")
        try:
            # 对每个批次无人机单独进行运算
            while self.epoch < self.epochNum-1:

                # 更具任务状态分配任务
                self.inControl()

                self.epoch += 1
        except Exception as e:
            print(traceback.print_exc()) # debug exception