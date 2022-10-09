# -*- coding: UTF-8 -*-
#!/usr/bin/env python
from multiprocessing import Process
import numpy as np
from enum import Enum
import traceback # 错误堆栈

# 自定义库
from algorithms.angle_coverage.func import Func

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
# brokenIndex = [5, 10]
brokenIndex = []

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
        # 判断无人机是否参与覆盖，参与赋值1，不参与覆盖
        activate = np.array([True if status == Status.Cover else False for status in self.flightStatus])

        # 初始化局部变量，避免频繁访问self造成时间成本过高
        veAngle = np.zeros(self.n)
        ue_hx = np.zeros(self.n)
        ue_hy = np.zeros(self.n)
        uc_hx = np.zeros(self.n)
        uc_hy = np.zeros(self.n)
        u_hx = np.zeros(self.n)
        u_hy = np.zeros(self.n)

        # 仅有一个无人机在执行覆盖任务，直接前行
        if sum(activate) < 2:
            u_hx[activate] = vMax
        else:
            self.updateLossConn()
            # 只计算当前参与覆盖任务的无人机控制量
            ue = self.func.ccangle(
                self.positions[activate, :],
                self.Angle_h[activate, epoch],
                self.ue_hy[activate, epoch],
                self.veAngle_h[activate, epoch])

            ue_hx[activate] = vMax
            ue_hy[activate] = ue[:]

            # 分段控制
            features = np.ones(sum(activate)) * self.value[1]
            featureVec = self.vectors[:, 1]
            uc = self.func.con_pre(features, featureVec, self.positions[activate, :], self.d, self.A)

            uc[:, 1] = 10 * uc[:, 1]
            # 限幅
            for index in range(uc.shape[0]):
                dist = np.linalg.norm(uc[index, :])
                if dist > vMax:
                    uc[index, :] = vMax * uc[index, :] / dist

            uc_hx[activate] = uc[:, 0]
            uc_hy[activate] = uc[:, 1]

            # 总控制，y轴方向
            u_y = np.zeros(ue.shape)

            # 防止倒飞
            changeIndex = ue * (uc[:, 1] + ue) < 0
            u_y[changeIndex] = ue[changeIndex]
            u_y[~changeIndex] = (uc[~changeIndex, 1] + ue[~changeIndex])

            # 控制率叠加
            u_hx[activate] = vMax
            u_hy[activate] = u_y

            veAngle[activate] = np.real(np.arctan(u_hy[activate]/u_hx[activate]))

        # 非覆盖任务的无人机控制量判断
        for index, status in enumerate(self.flightStatus):
            Px, Py = self.positions[index, :]
            # 控制量为0，不更新任何信息
            if status == Status.Stay:
                pass
            if index % 2 == 1 and status == Status.Back:
                if Px < positionStart and Py <= 1.0:
                    u_hx[index] = vBack
                    u_hy[index] = 0
                elif Px < positionStart and Py >= 1.0:
                    u_hx[index] = 0
                    u_hy[index] = -vBack
                    veAngle[index] = -np.pi/2
                elif Px >= positionEnd and Py < 2.0:
                    u_hx[index] = 0
                    u_hy[index] = vBack
                    veAngle[index] = np.pi/2
                elif Px >= positionEnd and Py >= 2.0:
                    u_hx[index] = -vBack
                    u_hy[index] = 0
                    veAngle[index] = np.pi/2
                elif Px >= positionStart and Py >= 2.0:
                    u_hx[index] = -vBack
                    u_hy[index] = 0
                    veAngle[index] = np.pi
            elif index % 2 == 0 and status == Status.Back:
                if Px < positionStart and Py >= -1.0:
                    u_hx[index] = vBack
                    u_hy[index] = 0
                elif Px < positionStart and Py <= -1.0:
                    u_hx[index] = 0
                    u_hy[index] = vBack
                    veAngle[index] = -np.pi/2
                elif Px >= positionEnd and Py > -2.0:
                    u_hx[index] = 0
                    u_hy[index] = -vBack
                    veAngle[index] = np.pi/2
                elif Px >= positionEnd and Py < -2.0:
                    u_hx[index] = -vBack
                    u_hy[index] = 0
                    veAngle[index] = np.pi/2
                elif Px > positionStart and Py <= -2.0:
                    u_hx[index] = -vBack
                    u_hy[index] = 0
                    veAngle[index] = np.pi

        # 更新历史记录
        self.uc_hy[:, epoch + 1] = uc_hy
        self.uc_hx[:, epoch + 1] = uc_hx
        self.ue_hx[:, epoch + 1] = ue_hx
        self.ue_hy[:, epoch + 1] = ue_hy
        self.u_hx[:, epoch + 1] = u_hx
        self.u_hy[:, epoch + 1] = u_hy
        self.veAngle_h[:, epoch + 1] = veAngle

        # 更新无人机位置
        self.Px_h[:, epoch + 1] = self.Px_h[:, epoch] + self.u_hx[:, epoch + 1] * self.dt
        self.Py_h[:, epoch + 1] = self.Py_h[:, epoch] + self.u_hy[:, epoch + 1] * self.dt
        self.Angle_h[:, epoch + 1] = np.pi + np.arctan(
            (circleY - self.Py_h[:, epoch + 1]) / (circleX - self.Px_h[:, epoch + 1]))

        self.positions[:, 0] = self.Px_h[:, epoch + 1]
        self.positions[:, 1] = self.Py_h[:, epoch + 1]

        # 发布对应无人机执行情况
        for k in range(self.n):
            Px, Py = self.positions[k, :]
            self.res.put({
                "Px": Px,
                "Py": Py,
                "Id": self.IdList[k],
                "index": epoch,
                "ux": u_hx[k],
                "uy": u_hy[k],
                "uz": -0.3 if self.flightStatus[k] == status.Broken else 0
            })

    # 状态转移
    def updateStatus(self):
        for index in range(self.n):
            # 计算无人机开始飞行的时间节点
            startEpoch = (index % self.n) * interval / self.dt

            # 获取当前实时位置
            Px, Py = self.positions[index]

            # 无人机由静止开始覆盖任务
            if self.flightStatus[index] == Status.Stay and self.epoch >= startEpoch:
                self.flightStatus[index] = Status.Back
            # 无人机由覆盖开始返回
            elif self.flightStatus[index] == Status.Cover and Px > positionEnd:
                self.flightStatus[index] = Status.Back
            # 已返回覆盖区域
            elif self.flightStatus[index] == Status.Back and Px >= positionStart and Px <= positionEnd and Py < 2.0 and Py > -2.0:
                self.flightStatus[index] = Status.Cover
            elif self.flightStatus[index] == Status.Cover and self.epoch > 500 and index in brokenIndex:
                self.flightStatus[index] = Status.Broken


    def run(self):
        print("start calculating!")
        try:
            # 对每个批次无人机单独进行运算
            while self.epoch < self.epochNum-1:
                # 更新任务状态
                self.updateStatus()

                # 更具任务状态分配任务
                self.inControl()

                self.epoch += 1
        except Exception as e:
            print(traceback.print_exc()) # debug exception