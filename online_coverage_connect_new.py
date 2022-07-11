# -*- coding: UTF-8 -*-
#!/usr/bin/env python
from scipy.spatial.transform import Rotation
import sys
import os
import pickle
import yaml
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import argparse
from multiprocessing import Process, Queue
from enum import Enum

# if python3
# time.clock = time.time

# 添加路径
currentUrl = os.path.dirname(__file__)
parentUrl = os.path.abspath(os.path.join(currentUrl, os.pardir))
sys.path.append(parentUrl)

# 模拟参数 --local仅本地画图模拟
parser = argparse.ArgumentParser()
parser.add_argument("--local", help="Run using local simulation.", action="store_true")
parser.add_argument("--record", help="save the waypoints.", action="store_true")
parser.add_argument("--load", help="load waypoints from record.", action="store_true")
args = parser.parse_args()

if not args.local:
    # 无人机接口
    from pycrazyswarm import *

# 自定义库
from algorithms.connect_coverage_new.LaplaMat import L_Mat
from algorithms.connect_coverage_new.connect_preserve import con_pre
from algorithms.connect_coverage_new.ccangle import ccangle

# 无人机状态枚举
Status = Enum("Status", ("Stay", "Cover", "BackUp", "BackLeft", "BackDown"))

# 读取无人机位置配置
with open("online_simulation_dev/crazyfiles.yaml", "r") as f:
# with open("crazyfiles.yaml", "r") as f:
    data = yaml.load(f, Loader=yaml.FullLoader)
allCrazyFlies = data['files']

# 实验参数
STOP = False
r = 2.0 # 雷达半径 
circleX, circleY = 8.0, 0.5  # 雷达中心
angleStart, angleEnd = np.pi*165/180, np.pi*195/180  # 扇面覆盖范围30°
cov = 4/180*np.pi  # 单机覆盖角度

# 参数设置
R = 1.5  # 通信半径
delta = 0.1  # 通信边界边权大小，越小效果越好
epsilon = 0.1  # 最小代数连通度
interval = 20 # 批次出发时间间隔
vMax = 0.2  # 连通保持最大速度（用于限幅）
vc_Max =  0.01 # connect speed limit
totalTime = 1000  # 仿真总时长
dt = 0.1  # 控制器更新频率
epochNum = int(np.floor(totalTime / dt))
draw = False # 是否画图
Z = 0.5 # 高度
calculTimeOut = 10 # 每轮运算超时设定

class workers(Process):
    def __init__(self, name, res, allCrazyFlies):
        Process.__init__(self)
        self.res = res
        self.name = name
        self.epoch = 0
        self.getParams(allCrazyFlies)

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
    def update_loss_conn(self):
        L, A, d = L_Mat(self.positions, R, delta)

        value, vectors = np.linalg.eig(L)
        # 从小到大对特征值进行排序
        index = np.argsort(value)
        vectors = vectors[:, index]
        value = value[index]
        # 第二小
        self.lambda_h[self.epoch] = value[1]

        self.L = L
        self.A = A
        self.d = d

        print(f"时刻{self.epoch}的连通度为{self.lambda_h}")

    # 用于初始化参数储存空间
    def storage_init(self):
        # 历史储存大小
        shape = (self.n, epochNum)

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
        lambda_h = np.zeros(epochNum)

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
        self.veAngle_h = veAngle_h
        self.lambda_h = lambda_h

    def inControl(self):
        epoch = self.epoch
        activate = np.ones(self.n) # 判断无人机是否参与覆盖，参与赋值1，不参与复制0

        ue = ccangle(
            self.positions[:, :],
            self.Angle_h[:, epoch],
            self.ue_hy[:, epoch],
            self.veAngle_h[:, epoch],
            angleStart, angleEnd, R, vMax, cov)

        # break
        self.ue_hx[:, epoch + 1] = ue[:, 0]
        self.ue_hy[:, epoch + 1] = ue[:, 1]

        # 判断无人机控制率是否改变，使无人机轨迹平滑
        # print(np.abs(ue_hx[:, epoch+1] - ue_hx[:,epoch]))
        changeIndex = np.abs(self.ue_hx[:, epoch + 1] - self.ue_hx[:, epoch]) < 0.0001
        self.ue_hx[changeIndex, epoch + 1] = self.ue_hx[changeIndex, epoch]
        changeIndex = np.abs(self.ue_hy[:, epoch + 1] - self.ue_hy[:, epoch]) < 0.0001
        self.ue_hy[changeIndex, epoch + 1] = self.ue_hy[changeIndex, epoch]
        # 分段控制
        features = np.ones(self.n) * value[1]
        featureVec = vectors[:, 1]
        uc = con_pre(features, featureVec, self.positions[:, :], d, A, R, delta, epsilon)
        # 限幅
        for agent in range(self.n):
            dist = np.linalg.norm(uc[agent, :])
            if dist > vc_Max:
                uc[agent, :] = vc_Max * uc[agent, :] / dist
        self.uc_hx[:, epoch + 1] = uc[:, 0]
        self.uc_hy[:, epoch + 1] = uc[:, 1]

        # 总控制
        u = 0.3 * uc + ue

        # 控制率叠加
        self.u_hx[:, epoch + 1] = u[:, 0]
        self.u_hy[:, epoch + 1] = u[:, 1]
        self.Px_h[:, epoch + 1] = self.Px_h[:, epoch] + u[:, 0] * dt
        self.Py_h[:, epoch + 1] = self.Py_h[:, epoch] + u[:, 1] * dt
        self.Angle_h[:, epoch + 1] = np.pi + np.arctan(
            (circleY - self.Py_h[:, epoch + 1]) / (circleX - self.Px_h[:, epoch + 1]))
        Angle = self.Angle_h[:, epoch + 1]

        changeIndex = self.u_hy[:, epoch + 1] > vMax
        self.u_hy[changeIndex, epoch + 1] = vMax

        self.veAngle_h[:, epoch + 1] = np.arcsin(self.u_hy[:, epoch + 1] / vMax)
        # 判断无人机是否执行覆盖任务
        changeIndex = self.Px_h[:, epoch] <= -2.5
        activate[changeIndex] = 0
        self.u_hx[changeIndex, epoch + 1] = self.u_hx[changeIndex, epoch]
        self.u_hy[changeIndex, epoch + 1] = self.u_hy[changeIndex, epoch]
        self.Px_h[changeIndex, epoch + 1] = self.Px_h[changeIndex, epoch] + self.u_hx[changeIndex, epoch + 1] * dt
        self.Py_h[changeIndex, epoch + 1] = self.Py_h[changeIndex, epoch] + self.u_hy[changeIndex, epoch + 1] * dt
        self.Angle_h[changeIndex, epoch + 1] = np.pi + np.arctan(
            (circleY - self.Py_h[changeIndex, epoch + 1]) / (circleX - self.Px_h[changeIndex, epoch + 1]))
        Angle[changeIndex] = self.Angle_h[changeIndex, epoch + 1]
        self.veAngle_h[changeIndex, epoch + 1] = np.arcsin(self.u_hy[changeIndex, epoch + 1] / vMax)
        #更新位置
        self.positions[:, 0] = self.Px_h[:, epoch + 1]
        self.positions[:, 1] = self.Py_h[:, epoch + 1]
        # 发布对应无人机执行情况
        for k in range(self.n*self.n):
            Px, Py = self.positions[k, :]
            self.res.put({
                "Px": Px,
                "Py": Py,
                "Id": self.IdList[k],
                "theta": 0, # 角度信息
                "index": epoch,
                "ux": self.u_hx[k, epoch + 1],
                "uy": self.u_hy[k, epoch + 1]
            })

        # 计算下一时刻的连通度
        L, A, d = L_Mat(self.positions[:, :], R, delta)
        value, vectors = np.linalg.eig(L)
        # 从小到大对特征值进行排序
        index = np.argsort(value)
        vectors = vectors[:, index]
        value = value[index]

        print("{}时刻的连通度为{}".format(epoch + 1, value[1]))
        self.lambda_h[epoch+1] = value[1]

    # 状态转移
    def checkStatus(self):
        for index in range(self.n):
            # 计算无人机开始飞行的时间节点
            startEpoch = (index % self.n) * interval / dt

            # 获取当前实时位置
            Px, Py = self.positions[index]
            ux, uy = 0, 0

            # 无人机由静止开始覆盖任务
            if self.flightStatus[index] == Status.Stay and self.epoch >= startEpoch:
                self.flightStatus[index] = Status.Cover
                continue
            # 无人机静止
            elif self.flightStatus[index] == Status.Stay:
                ux, uy = 0, 0
            # 无人机由覆盖开始返回
            elif self.flightStatus[index] == Status.Cover and Px > 2.5:
                self.flightStatus[index] = Status.Back
                ux, uy = 0, vMax
            elif self.flightStatus[index] == Status.Back and Px > 2.5 and Py < 2:
                ux, uy = 0, vMax
            elif self.flightStatus[index] == Status.Back and Py > 2 and Px > -2:
                ux, uy = -vMax, 0
            elif self.flightStatus[index] == Status.Back and Px < -2 and Py > -1.5:
                ux, uy = 0, -vMax
            # 只要有一个满足该条件，该批次所有无人机都重新开始覆盖任务
            elif self.flightStatus[index] == Status.Back and Px < -2 and Py < -1.5:
                self.flightStatus[index] = [Status.Cover] * self.n
                continue

            self.res.put({
                    "Px": Px,
                    "Py": Py,
                    "Id": self.IdList[index],
                    "theta": 0, # 角度信息
                    "index": self.epoch,
                    "ux": ux,
                    "uy": uy
                })

    def run(self):
        print("start calculating!")
        try:
            # 对每个批次无人机单独进行运算
            while self.epoch < epochNum:
                self.update_loss_conn()

                # 根据无人机位置判断当前执行任务
                self.checkStatus()
                for index in range(self.batch):
                    # 如果当前批次的所有无人机都处于覆盖状态
                    if all(self.flightStatus[index * self.n: (index+1) * self.n] == Status.Cover):
                        # 更新位置，计算控制率
                        self.inControl(index)
                self.epoch += 1
        except Exception as e:
            print(e)

if __name__ == '__main__':
    allWaypoints = []

    # --load从本地文件直接读取路径结果
    if args.load:
        f = open("record.txt", "rb")
        allWaypoints = pickle.load(f)
        f.close()
    else:
        # allWaypoints = getWaypoint()
        resultStorage = Queue()
        process = workers('Process1', resultStorage, allCrazyFlies)
        # 将进程设置为守护进程，当主程序结束时，守护进程会被强行终止
        process.daemon = True
        process.start()

    # --record, 记录路径结果到本地txt文件，方便直接读取
    if args.record:
        f = open("record.txt", "wb")
        pickle.dump(allWaypoints, f)
        f.close()

    if not args.local:
        framRate = 1.0 / dt

        print('Start flying!')

        # 创建无人机实例
        swarm = Crazyswarm()
        timeHelper = swarm.timeHelper
        allcfs = swarm.allcfs

        # 所有无人机同时起飞
        allcfs.takeoff(targetHeight=Z, duration=1.0)
        # 等待2秒
        timeHelper.sleep(2.0)

        # 修正系数
        kPosition = 1.
        # 获取无人机字典
        allcfsDict = allcfs.crazyfliesById

        executeNumber = 0

        while not resultStorage.empty():
            waypoint = resultStorage.get()
            # 取出实际位置和速度
            vx = waypoint['ux']
            vy = waypoint['uy']
            desiredPos = np.array([waypoint['Px'], waypoint['Py'], Z])

            # 获取对应ID的无人机控制器实例positions
            cf = allcfsDict[waypoint['Id']]

            actualPosition = cf.position()
            quaternion = cf.quaternion()

            rot = Rotation.from_quat(quaternion)
            actualPose = rot.as_euler("xyz")
            error = desiredPos - actualPosition

            cf.cmdVelocityWorld(np.array([vx, vy, 0] + kPosition * error), yawRate = 0)

            executeNumber += 1
            if(executeNumber == len(allCrazyFlies)):
                timeHelper.sleepForRate(framRate)
                executeNumber = 0

        print('Land!')
        # print2txt(json.dumps(self.logBuffer))
        # print('saved data')
        allcfsDict = allcfs.crazyfliesById
        cfs = allcfsDict.values()
        i = 0
        while True:
            i=i+1
            for cf in cfs:
                current_pos=cf.position()
                if current_pos[-1]>0.05:
                    vx=0
                    vy=0
                    vz=-0.3
                    cf.cmdVelocityWorld(np.array([vx, vy, vz] ), yawRate=0)
                    timeHelper.sleepForRate(framRate)
                else:
                    cf.cmdStop()
                    cfs.remove(cf)
            if len(cfs)==0:
                    break
