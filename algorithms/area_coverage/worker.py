# -*- coding: UTF-8 -*-
#!/usr/bin/env python
from multiprocessing import Process
import numpy as np
from enum import Enum
import traceback # 错误堆栈

# 自定义库
from algorithms.area_coverage.func import Func

# 参数配置
r = 2.0 # 雷达半径
circleX, circleY = 6.0, 0.0  # 雷达中心
angleStart, angleEnd = np.pi*165/180, np.pi*195/180  # 扇面覆盖范围30°
cov = 5/180*np.pi  # 单机覆盖角度
# 覆盖范围
positionStart = -2.5
positionEnd = 3.0

# 参数设置
R = 10  # 通信半径
delta = 0.1  # 通信边界边权大小，越小效果越好
epsilon = 0.1  # 最小代数连通度
interval = 2.0 # 批次出发时间间隔
vMax = 0.5  # 连通保持最大速度（用于限幅）
vBack = 0.6 # 无人机返回速度

# 无人机状态枚举
Status = Enum("Status", ("Stay", "Cover", "Back", "Broken"))

class Workers(Process):
    def __init__(self, name, res, allCrazyFlies, dt, epochNum):
        Process.__init__(self)
        self.res = res
        self.name = name
        self.epoch = 0
        self.dt = dt
        self.epochNum = epochNum
        self.getParams(allCrazyFlies)
        self.func = Func(positionStart, positionEnd, angleStart, angleEnd, 
        R, vMax, cov, delta, epsilon)

    # 从配置文件中解析无人机相关参数
    def getParams(self, allCrazyFlies):
        self.IdList = [item['Id'] for item in allCrazyFlies]

        self.n = len(self.IdList) # 无人机数量

        # 批次无人机状态
        self.flightStatus = [Status.Stay] * len(allCrazyFlies)

        # 转换为numpy数组
        self.positions = np.array([item['Position'] for item in allCrazyFlies])

        # 计算角度信
        self.angles = np.pi + np.arctan((circleY - self.positions[:, 1]) / (circleX - self.positions[:, 0]))

        self.storage_init()

    # 更新损失和连通度
    def updateLossConn(self):
        activate = np.array([True if status == Status.Cover else False for status in self.flightStatus])
        L, A, d = self.func.L_Mat(self.positions[activate, :])

        value, vectors = np.linalg.eig(L)
        # 从小到大对特征值进行排序
        index = np.argsort(value)
        self.vectors = vectors[:, index]
        self.value = value[index]

        # 第二小
        self.lambda_h[self.epoch] = self.value[1]

        self.L = L
        self.A = A
        self.d = d

        print("时刻" + str(self.epoch)+ " 的连通度为" + str(self.value[1]))

    # 用于初始化参数储存空间
    def storage_init(self):
        # 历史储存大小
        shape = (self.n, self.epochNum)

        # 无人机位置，角度数据保存
        Px_h = np.zeros(shape)
        Py_h = np.zeros(shape)
        Angle_h = np.zeros(shape)

        # 日志向量定义（记录控制量）
        ue_hx = np.zeros(shape)
        ue_hy = np.zeros(shape)
        uc_hx = np.zeros(shape)
        uc_hy = np.zeros(shape)
        u_hx = np.zeros(shape)
        u_hy = np.zeros(shape)
        veAngle_h = np.zeros(shape)
        lambda_h = np.zeros(self.epochNum)

        # 首值初始化
        Px_h[:, 0] = self.positions[:, 0]
        Py_h[:, 0] = self.positions[:, 1]
        Angle_h[:, 0] = self.angles[:]
        ue_hx[:, 0] = vMax
        u_hx[:, 0] = ue_hx[:, 0]
        u_hy[:, 0] = ue_hy[:, 0]

        # 挂载
        self.Px_h = Px_h
        self.Py_h = Py_h
        self.Angle_h = Angle_h
        self.ue_hx = ue_hx
        self.ue_hy = ue_hy
        self.uc_hx = uc_hx
        self.uc_hy = uc_hy
        self.u_hx = u_hx
        self.u_hy = u_hy
        self.veAngle_h = veAngle_h
        self.lambda_h = lambda_h

    def inControl(self):
        epoch = self.epoch

        # 初始化局部变量，避免频繁访问self造成时间成本过高
        veAngle = np.zeros(self.n)
        ue_hx = np.zeros(self.n)
        ue_hy = np.zeros(self.n)
        uc_hx = np.zeros(self.n)
        uc_hy = np.zeros(self.n)
        u_hx = np.zeros(self.n)
        u_hy = np.zeros(self.n)

        self.updateLossConn()
        # 连通控制量
        ue = self.func.con_control(self.positions)

        # 限幅


        features = np.ones(self.n) * self.value[1]
        featureVec = self.vectors[:, 1]
        # 覆盖控制量
        uc = self.func.con_pre(features, featureVec, self.positions, self.d, self.A)

        # 限幅
        for index in range(uc.shape[0]):
            dist = np.linalg.norm(uc[index, :])
            if dist > 1.2 * vMax:
                uc[index, :] = 1.2 * vMax * uc[index, :] / dist

        u = np.zeros(uc.shape)

        if self.value[1] >= self.warnEpsilon:
            u = ue[:]
        else:
            if t % T == 0 and t <= 70:
                total_ue = np.sum(ue, axis=0)

                norm = np.linalg.norm(total_ue)

                if norm  > 3 * vMax:
                    total_ue = 3 * vMax * total_ue / norm

                u = total_ue
            else:
                u = ue + uc

        position, waypinsts = matchPoints(position, u)

        # 发布对应无人机执行情况
        for k in range(self.n):
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