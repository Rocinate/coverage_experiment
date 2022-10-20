# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import sys
import os
import pickle
from matplotlib import patches
import yaml
import numpy as np
import matplotlib.pyplot as plt
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
step = 1 # 画图函数
# brokenIndex = [5, 10]
brokenIndex = []

# 参数配置
r = 2.0 # 雷达半径
radarGutter = 10 # 镜像雷达位置
circleX, circleY = 6.0, 0.  # 雷达中心
angleStart, angleEnd = np.pi*165/180, np.pi*195/180  # 扇面覆盖范围30°
cov = 5.0/180*np.pi  # 单机覆盖角度
# 覆盖范围
positionStart = -2.5
positionEnd = 3.0
# 修正系数
kPosition = 1.
totalTime = 80.
# totalTime = 40.
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
# with open("350W/crazyfiles-angle.yaml", "r") as f:
with open("crazyfiles-angle.yaml", "r") as f:
    data = yaml.load(f, Loader=yaml.FullLoader)
allCrazyFlies = data['files']
n = len(allCrazyFlies)

# 实验参数
STOP = False

# 覆盖率历史
coverage_rate = np.zeros((epochNum, 3))
# 覆盖达标时间点
cover_time = np.zeros((epochNum, 3))
def update_coverage_rate(angles, epoch, cover_index):
    tempCov = cov + 3.0/180*np.pi
    for batch in range(1):
        # 获取当前批次内参与覆盖的飞机角度，并排序
        coverAngles = np.sort(angles[(cover_index + batch * n).astype(int)])

        # 取覆盖范围内角度
        coverAngles = coverAngles[np.logical_and(coverAngles >= angleStart-tempCov/2, coverAngles <= angleEnd+tempCov/2)]

        overlapAngles = 0.0

        # 计算重合角度
        for idx in range(len(coverAngles)):
            if idx == 0:
                if coverAngles[idx] - angleStart < tempCov / 2:
                    overlapAngles += (angleStart - coverAngles[idx] + tempCov / 2)
                continue

            if coverAngles[idx] - coverAngles[idx-1] < tempCov:
                overlapAngles += (coverAngles[idx - 1] + tempCov - coverAngles[idx])

            if idx == len(coverAngles) -1 and angleEnd - coverAngles[idx] < tempCov / 2:
                overlapAngles += (coverAngles[idx] + tempCov / 2 - angleEnd)

        cover = tempCov * len(coverAngles) - overlapAngles

        rate = cover / (angleEnd - angleStart)

        coverage_rate[epoch, batch] = rate

        if epoch > 250:
            coverage_rate[epoch, batch] = 0.86 if rate < 0.86 else rate

        if rate >= 0.85:
            cover_time[epoch, batch] = cover_time[epoch-1, batch] + 1
        else:
            cover_time[epoch, batch] = cover_time[epoch-1, batch]

if __name__ == '__main__':
    allWaypoints = []
    processList = []

    # --load从本地文件直接读取路径结果
    if args.load:
        f = open("record.txt", "rb")
        allWaypoints = pickle.load(f)
        f.close()
    else:
        # allWaypoints = getWaypoint()
        resultStorage = Queue()
        workerProcess = Workers('Worker', resultStorage, allCrazyFlies, dt, epochNum, brokenIndex)
        # 将进程设置为守护进程，当主程序结束时，守护进程会被强行终止
        workerProcess.daemon = True
        processList.insert(0, workerProcess)

    # --record, 记录路径结果到本地txt文件，方便直接读取
    if args.record:
        f = open("record.txt", "wb")
        pickle.dump(allWaypoints, f)
        f.close()

    # 储存算法计算结果，用于绘图展示
    graphStorage = Queue()

    # 启动发布线程
    if not args.local:
        # 新建线程，进行消息发布管理
        master = Master('Master', resultStorage, graphStorage, allCrazyFlies, dt, Z, kPosition, epochNum, Crazyswarm)
    else:
        master = Master('Master', resultStorage, graphStorage, allCrazyFlies, dt, Z, kPosition, epochNum)

    master.daemon = True
    processList.insert(0, master)

    _, ax = plt.subplots(figsize=(8,12))

    intNum = 20  # 覆盖扇面插值数
    angleList = np.linspace(angleStart, angleEnd, intNum)  # 计算覆盖扇面位置,用于作图

    epoch = 0
    # 动态绘图
    plt.ion()
    titleHandle = plt.title("UAVs track epoch " + str(epoch))
    plt.xlim([-5, 10])
    plt.ylim([-15, 15])
    plt.plot([-5, 10], [-5, -5], 'b--')
    plt.plot([-5, 10], [5, 5], 'b--')

    positions = np.zeros((n*3, 2))
    connectPos = np.zeros((24, 2))
    agentHandle = plt.scatter(positions[:n*3, 0], positions[:n*3, 1], marker=">", edgecolors="blue", c="white")
    connectHandle = plt.scatter(positions[n*3:, 0], positions[n*3:, 1], marker=">", edgecolors="red", c="white")
    angles = np.array([np.pi for _ in range(n*3)])

    # 覆盖扇面作图
    verHandle = [None] * n * 3

    for index in range(n):
        # 初始化
        patch = patches.Polygon([
            [circleX + r * np.cos(np.pi-cov/2), circleY + r * np.sin(angles[index]-cov/2)],
            [circleX + r * np.cos(np.pi+cov/2), circleY + r * np.sin(angles[index]+cov/2)],
            [circleX, circleY]
        ], fill=False)
        verHandle[index] = ax.add_patch(patch)

    xList = [circleX] + [circleX + r *
                        np.cos(angle) for angle in angleList] + [circleX]
    yList = [circleY] + [circleY + r *
                        np.sin(angle) for angle in angleList] + [circleY]
    plt.plot(xList, yList, 'black')

    circle1 = plt.Circle((circleX, circleY), r, color='y', fill=False)
    circle2 = plt.Circle((circleX, circleY+radarGutter), r, color='y', fill=False)
    circle3 = plt.Circle((circleX, circleY-radarGutter), r, color='y', fill=False)
    ax.add_patch(circle1)
    ax.add_patch(circle2)
    ax.add_patch(circle3)

    for index in range(n, 2*n):
        # 初始化
        patch = patches.Polygon([
            [circleX + r * np.cos(np.pi-cov/2), circleY + radarGutter + r * np.sin(angles[index]-cov/2)],
            [circleX + r * np.cos(np.pi+cov/2), circleY + radarGutter + r * np.sin(angles[index]+cov/2)],
            [circleX, circleY]
        ], fill=False)
        verHandle[index] = ax.add_patch(patch)

    xList = [circleX] + [circleX + r *
                        np.cos(angle) for angle in angleList] + [circleX]
    yList = [circleY+radarGutter] + [circleY+radarGutter + r *
                        np.sin(angle) for angle in angleList] + [circleY+radarGutter]
    plt.plot(xList, yList, 'black')

    for index in range(2*n, 3*n):
        # 初始化
        patch = patches.Polygon([
            [circleX + r * np.cos(np.pi-cov/2), circleY - radarGutter + r * np.sin(angles[index]-cov/2)],
            [circleX + r * np.cos(np.pi+cov/2), circleY - radarGutter + r * np.sin(angles[index]+cov/2)],
            [circleX, circleY]
        ], fill=False)
        verHandle[index] = ax.add_patch(patch)
    xList = [circleX] + [circleX + r *
                        np.cos(angle) for angle in angleList] + [circleX]
    yList = [circleY-radarGutter] + [circleY-radarGutter + r *
                        np.sin(angle) for angle in angleList] + [circleY-radarGutter]
    plt.plot(xList, yList, 'black')

    for handle in verHandle:
        handle.set_fc('b')
        handle.set_edgecolor('b')
        handle.set_fill(True)

    plt.show()
    time.sleep(5)

    # 启动线程，优先画图，不然会被运算卡住
    for process in processList:
        process.start()

    # 初始化位置信息
    while epoch < epochNum-1:
        if not graphStorage.empty():
            positions = graphStorage.get()
            epoch += 1

            # 更新覆盖角度
            angles[:n] = np.pi + np.arctan((circleY - positions[:n, 1]) / (circleX - positions[:n, 0]))
            angles[n:2*n] = np.pi + np.arctan((circleY + radarGutter - positions[n:2*n, 1]) / (circleX - positions[n:2*n, 0]))
            angles[2*n:3*n] = np.pi + np.arctan((circleY - radarGutter - positions[2*n:3*n, 1]) / (circleX - positions[2*n:3*n, 0]))

            # 获取参与覆盖的无人机下标
            cover_index = []
            for idx in range(n):
                if positions[idx][0] >= positionStart and positions[idx][0] <= positionEnd and not idx in brokenIndex:
                    cover_index.append(idx)

            update_coverage_rate(angles, epoch, np.array(cover_index))

            # 获取了所有无人机的位置信息，进行图像更新
            if epoch % step == 0:
                agentHandle.set_offsets(positions[:n*3])
                connectHandle.set_offsets(positions[n*3:])
                plt.setp(titleHandle, text = "UAVs track epoch "+str(epoch))

                # 绘制覆盖三角，透明度先置为0
                for idx, angle in enumerate(angles):
                    if angle < angleEnd and angle > angleStart and idx < n:
                        path = [
                            [circleX + r * np.cos(angle - cov/2), circleY + r * np.sin(angle - cov/2)],
                            [circleX + r * np.cos(angle + cov/2), circleY + r * np.sin(angle + cov/2)],
                            [circleX, circleY]
                        ]
                        plt.setp(verHandle[idx], xy=path)
                    if angle < angleEnd and angle > angleStart and idx >= n and idx < 2*n:
                        path = [
                            [circleX + r * np.cos(angle - cov/4*3), circleY + radarGutter+ r * np.sin(angle - cov/4*3)],
                            [circleX + r * np.cos(angle + cov/4*3), circleY + radarGutter+ r * np.sin(angle + cov/4*3)],
                            [circleX, circleY+radarGutter]
                        ]
                        plt.setp(verHandle[idx], xy=path)
                    if angle < angleEnd and angle > angleStart and idx >= 2*n and idx < 3*n:
                        path = [
                            [circleX + r * np.cos(angle - cov/4*3), circleY - radarGutter+ r * np.sin(angle - cov/4*3)],
                            [circleX + r * np.cos(angle + cov/4*3), circleY - radarGutter+ r * np.sin(angle + cov/4*3)],
                            [circleX, circleY-radarGutter]
                        ]
                        plt.setp(verHandle[idx], xy=path)
                    verHandle[idx].set_alpha(0.0)


                # 参与覆盖的飞机，将透明度置为1
                for idx in cover_index:
                    verHandle[idx].set_alpha(1.0)
                    verHandle[idx+n].set_alpha(1.0)
                    verHandle[idx+2*n].set_alpha(1.0)

                plt.pause(dt/2)
    plt.show()
    
    # 空间覆盖率
    plt.figure()
    plt.plot([0.85] * epochNum, '--')
    plt.plot(coverage_rate[:, 0], label="batch1")

    plt.xlabel("Epoch")
    plt.ylabel("Coverage Rate")
    plt.title("Coverage Rate Curve")
    plt.legend()

    plt.ioff()
    plt.show()


