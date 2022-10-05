# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import sys
import os
import pickle
import yaml
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import time
import argparse
from multiprocessing import Queue
from algorithms.angle_coverage.worker import Workers
from algorithms.angle_coverage.master import Master

# if python3
# time.clock = time.time

# 飞行参数
Z = 0.5 # 高度
dt = 0.1 # 控制器更新频率

# 参数配置
r = 2.0 # 雷达半径
radarGutter = 10 # 镜像雷达位置
circleX, circleY = 6.0, 0.  # 雷达中心
angleStart, angleEnd = np.pi*165/180, np.pi*195/180  # 扇面覆盖范围30°
cov = 5/180*np.pi  # 单机覆盖角度
# 覆盖范围
positionStart = -2.5
positionEnd = 3.0
# 修正系数
kPosition = 1.
totalTime = 80.
epochNum = int(np.floor(totalTime / dt))

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

# 读取无人机位置配置
# with open("stable_angle_coverage_control/crazyfiles.yaml", "r") as f:
with open("crazyfiles-angle.yaml", "r") as f:
    data = yaml.load(f, Loader=yaml.FullLoader)
allCrazyFlies = data['files']

# 实验参数
STOP = False

def startCrazySwarm():
    # 创建无人机实例
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    return allcfs, timeHelper

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
        process = Workers('Worker', resultStorage, allCrazyFlies, dt, epochNum)
        # 将进程设置为守护进程，当主程序结束时，守护进程会被强行终止
        process.daemon = True
        process.start()

    # --record, 记录路径结果到本地txt文件，方便直接读取
    if args.record:
        f = open("record.txt", "wb")
        pickle.dump(allWaypoints, f)
        f.close()

    # 储存算法计算结果，用于绘图展示
    graphStorage = Queue()

    # 启动发布线程
    if not args.local:
        # 与无人机集群建立联系，读取初始化信息
        allcfs, timeHelper = startCrazySwarm()
        # 新建线程，进行消息发布管理
        master = Master('Master', resultStorage, graphStorage, allCrazyFlies, dt, Z, kPosition, epochNum, allcfs, timeHelper)
    else:
        master = Master('Master', resultStorage, graphStorage, allCrazyFlies, dt, Z, kPosition, epochNum)

    master.daemon = True
    master.start()

    _, ax = plt.subplots(figsize=(8,12))

    intNum = 20  # 覆盖扇面插值数
    angleList = np.linspace(angleStart, angleEnd, intNum)  # 计算覆盖扇面位置,用于作图
    # 扇形点位，添加起点保证图像闭合
    # xList = [circleX] + [circleX + r *
    #                     np.cos(angle) for angle in angleList] + [circleX]
    # yList = [circleY] + [circleY + r *
    #                     np.sin(angle) for angle in angleList] + [circleY]

    epoch = 0
    # 动态绘图
    plt.ion()
    titleHandle = plt.title("UAVs track epoch " + str(epoch))
    plt.xlim([-5, 10])
    plt.ylim([-15, 15])

    n = len(allCrazyFlies)
    positions = np.zeros((n*3, 2))
    connectPos = np.zeros((24, 2))
    agentHandle = plt.scatter(positions[:n*3, 0], positions[:n*3, 1], marker=">", edgecolors="blue", c="white")
    connectHandle = plt.scatter(positions[n*3:, 0], positions[n*3:, 1], marker=">", edgecolors="red", c="white")
    angles = np.array([np.pi for _ in range(n*3)])

    # 覆盖扇面作图
    # verHandle = [None] * n * 3
    # for index in range(n):
    #     # 初始化
    #     patch = patches.Polygon([
    #         [circleX + r * np.cos(np.pi-cov/2), circleY + r * np.sin(angles[index]-cov/2)],
    #         [circleX + r * np.cos(np.pi+cov/2), circleY + r * np.sin(angles[index]+cov/2)],
    #         [circleX, circleY]
    #     ], fill=False)
    #     verHandle[index] = ax.add_patch(patch)

    # for index in range(n, 2*n):
    #     # 初始化
    #     patch = patches.Polygon([
    #         [circleX + r * np.cos(np.pi-cov/2), circleY + 10 + r * np.sin(angles[index]-cov/2)],
    #         [circleX + r * np.cos(np.pi+cov/2), circleY + 10 + r * np.sin(angles[index]+cov/2)],
    #         [circleX, circleY]
    #     ], fill=False)
    #     verHandle[index] = ax.add_patch(patch)

    # for index in range(2*n, 3*n):
    #     # 初始化
    #     patch = patches.Polygon([
    #         [circleX + r * np.cos(np.pi-cov/2), circleY -10 + r * np.sin(angles[index]-cov/2)],
    #         [circleX + r * np.cos(np.pi+cov/2), circleY -10 + r * np.sin(angles[index]+cov/2)],
    #         [circleX, circleY]
    #     ], fill=False)
    #     verHandle[index] = ax.add_patch(patch)

    plt.show()

    # 初始化位置信息
    while epoch < epochNum-1:
        if not graphStorage.empty():
            positions = graphStorage.get()
            epoch += 1

            # 获取了所有无人机的位置信息，进行图像更新

            # angles[n:2*n] = np.pi + np.arctan((circleY +10 - positions[n:2*n, 1]) / (circleX - positions[n:2*n, 0]))
            # angles[2*n:3*n] = np.pi + np.arctan((circleY - 10 - positions[2*n:3*n, 1]) / (circleX - positions[2*n:3*n, 0]))

            agentHandle.set_offsets(positions[:n*3])
            connectHandle.set_offsets(positions[n*3:])
            plt.setp(titleHandle, text = "UAVs track epoch "+str(epoch))

            # for idx, angle in enumerate(angles):
            #     if angle < angleEnd and angle > angleStart and idx < n:
            #         path = [
            #             [circleX + r * np.cos(angle - cov/2), circleY + r * np.sin(angle - cov/2)],
            #             [circleX + r * np.cos(angle + cov/2), circleY + r * np.sin(angle + cov/2)],
            #             [circleX, circleY]
            #         ]
            #         plt.setp(verHandle[idx], xy=path)
            #     if angle < angleEnd and angle > angleStart and idx >= n and idx < 2*n:
            #         path = [
            #             [circleX + r * np.cos(angle - cov/2), circleY +10+ r * np.sin(angle - cov/2)],
            #             [circleX + r * np.cos(angle + cov/2), circleY +10+ r * np.sin(angle + cov/2)],
            #             [circleX, circleY+10]
            #         ]
            #         plt.setp(verHandle[idx], xy=path)
            #     if angle < angleEnd and angle > angleStart and idx >= 2*n and idx < 3*n:
            #         path = [
            #             [circleX + r * np.cos(angle - cov/2), circleY -10+ r * np.sin(angle - cov/2)],
            #             [circleX + r * np.cos(angle + cov/2), circleY -10+ r * np.sin(angle + cov/2)],
            #             [circleX, circleY-10]
            #         ]
            #         plt.setp(verHandle[idx], xy=path)
            plt.pause(0.000000000001)
    plt.ioff()
    plt.show()
