# -*- coding: UTF-8 -*-
#!/usr/bin/env python
from multiprocessing import Process
import numpy as np
import traceback # 错误堆栈

# 自定义库
from algorithms.area_coverage.func import Func

# 参数设置
R = 4.5  # 通信半径
delta = 0.1  # 通信边界边权大小，越小效果越好
epsilon = 0.1  # 最小代数连通度
vMax = 0.5  # 连通保持最大速度（用于限幅）
virtualVMax = 1.0 # 虚拟联通保持最大速度
warnEpsilon = 0.8 # 连通度警戒值

class Workers(Process):
    def __init__(self, name, res, allCrazyFlies, dt, epochNum, field_strength, box, flightNumConfig, noConnect):
        Process.__init__(self)
        self.res = res
        self.name = name
        self.epoch = 0
        self.dt = dt
        self.epochNum = epochNum
        self.flightNumConfig = flightNumConfig
        self.getParams(allCrazyFlies)
        self.func = Func(R, vMax, delta, epsilon, box, field_strength)
        self.warnEpsilon = warnEpsilon
        self.minConnect = 9999
        self.noConnect = noConnect

    # 从配置文件中解析无人机相关参数
    def getParams(self, allCrazyFlies):
        self.IdList = [item['Id'] for item in allCrazyFlies]

        self.n = len(self.IdList) # 无人机数量

        # 转换为numpy数组
        self.positions = np.array([item['Position'] for item in allCrazyFlies])

        # 获取非边界智能体索引
        notGuard = np.ones(self.n)
        notGuard[self.flightNumConfig['real']:self.flightNumConfig['real'] + self.flightNumConfig['guard']] = 0
        self.notGuard = notGuard.astype(np.bool)
        self.notGuardIndex = np.array([index for index, flag in enumerate(self.notGuard) if flag])

        # 历史连通度
        self.his = np.zeros(self.epochNum)

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

        self.his[self.epoch] = self.value[1]
        print("时刻" + str(self.epoch)+ " 的连通度为" + str(round(self.value[1], 2)))

        self.minConnect = self.minConnect if self.value[1] > self.minConnect else self.value[1]

    def inControl(self):
        epoch = self.epoch

        self.updateLossConn()
        # 连通控制量
        ue = self.func.con_control(self.positions)
        for index in range(self.flightNumConfig['real']):
            dist = np.linalg.norm(ue[index, :])
            if dist > vMax:
                ue[index, :] = vMax * ue[index, :] / dist

        # 计算边缘飞机索引
        guardIndex = self.flightNumConfig['real']+self.flightNumConfig['guard']
        for index in range(guardIndex, self.n):
            dist = np.linalg.norm(ue[index, :])
            if dist > virtualVMax:
                ue[index, :] = virtualVMax * ue[index, :] / dist

        features = np.ones(self.n) * self.value[1]
        featureVec = self.vectors[:, 1]
        # 覆盖控制量
        uc = self.func.con_pre(features, featureVec, self.positions, self.d, self.A)

        # 限幅
        for index in range(uc.shape[0]):
            dist = np.linalg.norm(uc[index, :])
            if dist > vMax:
                uc[index, :] = vMax * uc[index, :] / dist

        for index in range(guardIndex, self.n):
            dist = np.linalg.norm(uc[index, :])
            if dist > virtualVMax:
                ue[index, :] = virtualVMax * uc[index, :] / dist

        # 总控制率
        u = np.zeros(uc.shape)
        # 根据参数判断是否保持连通
        if self.noConnect:
            u = ue
        else:
            critical = self.func.is_Critical_robot(self.d, 0.7)

            for flightIndex in range(u.shape[0]):
                if self.value[1] <= self.warnEpsilon:
                    if critical[flightIndex]:
                        u[flightIndex] = 0.6*ue[flightIndex] + uc[flightIndex]
                    else:
                        u[flightIndex] = ue[flightIndex] + 0.5 * uc[flightIndex]
                else:
                    u[flightIndex] = ue[flightIndex]

        # 仅更新非边界智能体
        self.positions[self.notGuard] += u[self.notGuard] * self.dt

        # 发布智能体执行情况
        for index in self.notGuardIndex:
            # if flag:
            Px, Py = self.positions[index, :]
            self.res.put({
                "Px": Px,
                "Py": Py,
                "Id": self.IdList[index],
                "index": epoch,
                "ux": u[index, 0],
                "uy": u[index, 1],
                "uz": 0
            })

    def run(self):
        print("start calculating!")
        try:
            # 对每个批次无人机单独进行运算
            while self.epoch < self.epochNum:

                # 更具任务状态分配任务
                self.inControl()

                self.epoch += 1
            if self.minConnect > 0:
                print( "集群运行过程中最小连通度为" + str(round(self.minConnect, 2))+ ", 拓扑保持连通，满足指标要求")
            else:
                print( "集群运行过程中最小连通度为" + str(round(self.minConnect, 2)))

            self.res.put(self.his.copy())

        except Exception as e:
            print(traceback.print_exc()) # debug exception
