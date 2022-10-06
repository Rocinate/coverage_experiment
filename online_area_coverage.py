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
from algorithms.area_coverage.worker import Workers
from algorithms.area_coverage.master import Master

# if python3
# time.clock = time.time

# 飞行参数
Z = 0.5 # 高度
dt = 0.1 # 控制器更新频率

# 参数配置
r = 2.0 # 雷达半径
radarGutter = 10 # 镜像雷达位置
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
with open("crazyfiles-area.yaml", "r") as f:
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

    epoch = 0
    # 动态绘图
    plt.ion()
    titleHandle = plt.title("UAVs track epoch " + str(epoch))
    plt.xlim([-5, 10])
    plt.ylim([-15, 15])

    n = len(allCrazyFlies)
    positions = np.zeros((n, 2))
    connectPos = np.zeros((24, 2))
    agentHandle = plt.scatter(positions[:, 0], positions[:, 1], marker=">", edgecolors="blue", c="white")

    plt.show()

    # 初始化位置信息
    while epoch < epochNum-1:
        if not graphStorage.empty():
            positions = graphStorage.get()
            epoch += 1

            # 获取了所有无人机的位置信息，进行图像更新
            agentHandle.set_offsets(positions[:n])
            plt.setp(titleHandle, text = "UAVs track epoch "+str(epoch))

            plt.pause(0.000000000001)
    plt.ioff()
    plt.show()
