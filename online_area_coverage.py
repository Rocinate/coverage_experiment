# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import sys
import os
import pickle
import yaml
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import argparse
from scipy import interpolate
from multiprocessing import Queue
from algorithms.area_coverage.worker import Workers
from algorithms.area_coverage.master import Master

# if python3
# time.clock = time.time

# 飞行参数
Z = 0.5 # 高度
dt = 0.2 # 控制器更新频率
step = 5 # 画图函数

# 修正系数
kPosition = 1.
totalTime = 100.
epochNum = int(np.floor(totalTime / dt))

# xRange_min = -3.2  # 场地长度
# xRange_max = 3.8
# yRange_min = -3.2
# yRange_max = 3.8

# 欲覆盖范围
xRange_min = -3.2  # 场地长度
xRange_max = 10.0
yRange_min = -3.2
yRange_max = 10.0

box = np.array([xRange_min, xRange_max, yRange_min, yRange_max])  # 场地范围

# 添加路径
currentUrl = os.path.dirname(__file__)
parentUrl = os.path.abspath(os.path.join(currentUrl, os.pardir))
sys.path.append(parentUrl)

# 模拟参数 --local仅本地画图模拟
parser = argparse.ArgumentParser()
parser.add_argument("--local", help="Run using local simulation.", action="store_true")
parser.add_argument("--record", help="save the waypoints.", action="store_true")
parser.add_argument("--load", help="load waypoints from record.", action="store_true")
parser.add_argument("--noConnect", help="with connection.", action="store_true")
args = parser.parse_args()

if not args.local:
    # 无人机接口
    from pycrazyswarm import *

# 读取无人机位置配置
# with open("350W/crazyfiles-area.yaml", "r") as f:
with open("crazyfiles-area.yaml", "r") as f:
    data = yaml.load(f, Loader=yaml.FullLoader)
allCrazyFlies = data['files']

flightNumConfig = {
    "real": 12,
    "guard": 14,
    "virtual": len(allCrazyFlies) - 12 -14
}

if __name__ == '__main__':
    # 导入信号场数据
    field_strength = np.loadtxt(open('./devTools/cq.csv'), delimiter=',', skiprows=0, dtype=np.float64)  
    # field_strength = np.loadtxt(open('./350W/devTools/cq.csv'), delimiter=',', skiprows=0, dtype=np.float64)

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
        process = Workers('Worker', resultStorage, allCrazyFlies, dt, epochNum, field_strength, box, flightNumConfig, args.noConnect)
        # 将进程设置为守护进程，当主程序结束时，守护进程会被强行终止
        process.daemon = True
        processList.insert(0, process)

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
        master = Master('Master', resultStorage, graphStorage, allCrazyFlies, dt, Z, kPosition, epochNum, flightNumConfig, Crazyswarm)
    else:
        master = Master('Master', resultStorage, graphStorage, allCrazyFlies, dt, Z, kPosition, epochNum, flightNumConfig)

    master.daemon = True
    processList.insert(0, master)

    _, ax = plt.subplots(figsize=(4, 3))

    epoch = 0
    # 动态绘图
    plt.ion()
    titleHandle = plt.title("UAVs track epoch " + str(epoch))
    plt.xlim([xRange_min, xRange_max])
    plt.ylim([yRange_min, yRange_max])

    # 绘制场强背景图
    grid_x, grid_y = np.mgrid[box[0]:box[1]:500j, box[2]:box[3]:500j]
    f = interpolate.griddata(field_strength[:, :2], field_strength[:,2], (grid_x, grid_y), method='linear')
    colors=["magenta","blueviolet","royalblue","aqua","springgreen","greenyellow","yellow","orangered","red","white"]
    clrmap=mcolors.LinearSegmentedColormap.from_list("mycmap",colors)
    plt.pcolor(grid_x,grid_y,f,cmap=clrmap)

    plt.plot([-3.2,3.2], [3.8,3.8], 'b--')
    plt.plot([3.2,3.2], [3.8,-3.8], 'b--')
    plt.plot([-3.2,3.2], [-3.8,-3.8], 'b--')
    plt.plot([-3.2,-3.2], [3.8,-3.8], 'b--')

    n = len(allCrazyFlies)
    positions = np.zeros((n-flightNumConfig['guard'], 2))
    trueAgentHandle = plt.scatter(positions[:flightNumConfig["real"], 0], positions[:flightNumConfig["real"], 1], marker=">", edgecolors="blue", c="white")
    fakeAgentHandle = plt.scatter(positions[-flightNumConfig["virtual"]:, 0], positions[-flightNumConfig["virtual"]:, 1], marker=">", edgecolors="blue", c="blue")

    plt.show()

    # 启动线程，优先画图
    for process in processList:
        process.start()

    # 初始化位置信息
    while epoch < epochNum-1:
        if not graphStorage.empty():
            positions = graphStorage.get()
            epoch += 1

            if(epoch % step == 0):
                trueAgentHandle.set_offsets(positions[:flightNumConfig["real"]])
                fakeAgentHandle.set_offsets(positions[-flightNumConfig["virtual"]:])

                plt.setp(titleHandle, text = "UAVs track epoch "+str(epoch))

                plt.pause(0.0000001)
    plt.ioff()
    plt.show()
