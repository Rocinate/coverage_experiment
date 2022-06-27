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
from algorithms.connect_coverage.LaplaMat import L_Mat
from algorithms.connect_coverage.connect_preserve import con_pre
from algorithms.connect_coverage.ccangle import ccangle


# 读取无人机位置配置
with open("online_simulation_dev/crazyfiles.yaml", "r") as f:
# with open("crazyfiles.yaml", "r") as f:
    data = yaml.load(f, Loader=yaml.FullLoader)
allCrazyFlies = data['files']
IdList = [item['Id'] for item in allCrazyFlies]
positions = np.array([item['Position'] for item in allCrazyFlies])

# 实验参数
STOP = False
r = 2.0 # 雷达半径 
circleX, circleY = 8.0, 0.5  # 雷达中心
angleStart, angleEnd = np.pi*165/180, np.pi*195/180  # 扇面覆盖范围30°
cov = 4/180*np.pi  # 单机覆盖角度

# 参数设置
R = 1.5  # 通信半径
n = len(allCrazyFlies)  # 无人机数量/batch
batch = 1  # 批次
delta = 0.1  # 通信边界边权大小，越小效果越好
epsilon = 0.1  # 最小代数连通度
vMax = 0.2  # 连通保持最大速度（用于限幅）
vc_Max =  0.01 # connect speed limit
veAngle = np.zeros(n) # 无人机朝向角
totalTime = 1000  # 仿真总时长
dt = 0.1  # 控制器更新频率
epochNum = int(np.floor(totalTime / dt))
draw = False # 是否画图
Z = 0.5 # 高度
calculTimeOut = 10 # 每轮运算超时设定

class workers(Process):
    def __init__(self, name, res):
        Process.__init__(self)
        self.res = res
        self.name = name
    
    def run(self):
        print("start calculating!")
        global positions
        try:
            # 无人机初始角度
            Angle = np.pi + np.arctan((circleY - positions[:, 1]) / (circleX - positions[:, 0]))
            # 无人机位置，角度数据保存
            Px_h = np.zeros((n*batch, epochNum))
            Py_h = np.zeros((n*batch, epochNum))
            Angle_h = np.zeros((n*batch, epochNum))
            Px_h[:, 0] = positions[:, 0]
            Py_h[:, 0] = positions[:, 1]
            Angle_h[:, 0] = Angle

            # 日志向量定义（记录控制量）
            ue_hx = np.zeros((n*batch, epochNum))
            ue_hy = np.zeros((n*batch, epochNum))
            ue_hx[:, 0] = vMax
            uc_hx = np.zeros((n*batch, epochNum))
            uc_hy = np.zeros((n*batch, epochNum))
            u_hx = np.zeros((n*batch, epochNum))
            u_hy = np.zeros((n*batch, epochNum))
            u_hx[:, 0] = ue_hx[:, 0] + uc_hx[:, 0]
            u_hy[:, 0] = ue_hy[:, 0] + uc_hy[:, 0]
            veAngle_h = np.zeros((n*batch, epochNum))
            lambda_h = np.zeros(epochNum)

            activate = np.ones(n) # 判断无人机是否参与覆盖，参与赋值1，不参与复制0

            # 初始损失和连通度计算
            L, A, d = L_Mat(positions, R, delta)

            value, vectors = np.linalg.eig(L)
            # 从小到大对特征值进行排序
            index = np.argsort(value)
            vectors = vectors[:, index]
            value = value[index]

            lambda_h[0] = value[1]
            print("1时刻的连通度为{}".format(value[1]))
            # plt.show()

            for epoch in range(epochNum):
                # print(value)
                if positions[:, 0].max() > 2.5:
                    break
                else:
                    # 角度覆盖控制率
                    activate = np.ones(n)
                    ue = ccangle(
                        positions,
                        Angle_h[:, epoch], ue_hy[:, epoch], veAngle_h[:, epoch],
                        angleStart, angleEnd, R, vMax, cov)
                    # print(ue)
                    # break
                    ue_hx[:, epoch + 1] = ue[:, 0]
                    ue_hy[:, epoch + 1] = ue[:, 1]

                    # 判断无人机控制率是否改变，使无人机轨迹平滑
                    # print(np.abs(ue_hx[:, epoch+1] - ue_hx[:,epoch]))
                    changeIndex = np.abs(ue_hx[:, epoch + 1] - ue_hx[:, epoch]) < 0.0001
                    ue_hx[changeIndex, epoch + 1] = ue_hx[changeIndex, epoch]
                    changeIndex = np.abs(ue_hy[:, epoch + 1] - ue_hy[:, epoch]) < 0.0001
                    ue_hy[changeIndex, epoch + 1] = ue_hy[changeIndex, epoch]
                    #分段控制
                    features = np.ones(n) * value[1]
                    featureVec = vectors[:, 1]
                    uc = con_pre(features, featureVec, positions, d, A, R, delta, epsilon)
                    # 限幅
                    for agent in range(n):
                        dist = np.linalg.norm(uc[agent, :])
                        if dist > vc_Max:
                            uc[agent, :] = vc_Max * uc[agent, :] / dist
                    uc_hx[:, epoch + 1] = uc[:, 0]
                    uc_hy[:, epoch + 1] = uc[:, 1]

                    # 总控制
                    # u = 3 * uc + ue
                    u = 0.3 * uc + ue

                    # 控制率叠加
                    u_hx[:, epoch + 1] = u[:, 0]
                    u_hy[:, epoch + 1] = u[:, 1]
                    Px_h[:, epoch + 1] = Px_h[:, epoch] + u[:, 0] * dt
                    Py_h[:, epoch + 1] = Py_h[:, epoch] + u[:, 1] * dt
                    Angle_h[:, epoch + 1] = np.pi + np.arctan(
                        (circleY - Py_h[:, epoch + 1]) / (circleX - Px_h[:, epoch + 1]))
                    Angle = Angle_h[:, epoch + 1]

                    changeIndex = u_hy[:, epoch + 1] > vMax
                    u_hy[changeIndex, epoch + 1] = vMax

                    veAngle_h[:, epoch + 1] = np.arcsin(u_hy[:, epoch + 1] / vMax)
                    # 判断无人机是否执行覆盖任务
                    changeIndex = Px_h[:, epoch] <= -2.5
                    activate[changeIndex] = 0
                    u_hx[changeIndex, epoch + 1] = u_hx[changeIndex, epoch]
                    u_hy[changeIndex, epoch + 1] = u_hy[changeIndex, epoch]
                    Px_h[changeIndex, epoch + 1] = Px_h[changeIndex, epoch] + u_hx[changeIndex, epoch + 1] * dt
                    Py_h[changeIndex, epoch + 1] = Py_h[changeIndex, epoch] + u_hy[changeIndex, epoch + 1] * dt
                    Angle_h[changeIndex, epoch + 1] = np.pi + np.arctan(
                        (circleY - Py_h[changeIndex, epoch + 1]) / (circleX - Px_h[changeIndex, epoch + 1]))
                    Angle[changeIndex] = Angle_h[changeIndex, epoch + 1]
                    veAngle_h[changeIndex, epoch + 1] = np.arcsin(u_hy[changeIndex, epoch + 1] / vMax)
                    #更新位置
                    positions[:, 0] = Px_h[:, epoch + 1]
                    positions[:, 1] = Py_h[:, epoch + 1]
                    for k in range(n*batch):
                        Px, Py = positions[k, :]
                        self.res.put({
                            "Px": Px,
                            "Py": Py,
                            "Id": IdList[k],
                            "theta": veAngle,
                            "index": epoch,
                            "ux": u_hx[k, epoch + 1],
                            "uy": u_hy[k, epoch + 1]
                        })

                    # 计算下一时刻的连通度
                    L, A, d = L_Mat(positions, R, delta)
                    value, vectors = np.linalg.eig(L)
                    # 从小到大对特征值进行排序
                    index = np.argsort(value)
                    vectors = vectors[:, index]
                    value = value[index]

                    print("{}时刻的连通度为{}".format(epoch + 1, value[1]))
                    lambda_h[epoch+1] = value[1]
        except Exception as e:
            # print(e)
            pass

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
        process = workers('Process1', resultStorage)
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
            if(executeNumber == n):
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
