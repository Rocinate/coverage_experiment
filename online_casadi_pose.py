# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import sys
import os
import yaml
import numpy as np
import time
import math
from multiprocessing import Process, Queue
import argparse
import pickle

# if python3
time.clock = time.time

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
    from utils.CFController import CFController

# 自定义库
from algorithms.borderdVoronoi import Vor
from algorithms.cassingle import Cassingle
from utils.graphController import Graph

# 读取无人机位置配置
# with open("online_simulation/crazyfiles.yaml", "r") as f:
with open("crazyfiles.yaml", "r") as f:
    data = yaml.load(f)
allCrazyFlies = data['files']

# 实验参数
STOP = False
numIterations = 25
xRange = 3.0
yRange = 2.0
box = np.array([-xRange, xRange, -yRange, yRange])  # 场地范围
lineSpeed = 0.1
angularSpeed = 0.2
draw =  True# 是否画图
T = 5.0
N = 10
allcfsTime = T/N
volume = 0.05
Z = 1.0 # 高度
processNum = len(allCrazyFlies) # 进程数，默认和无人机个数相同
calculTimeOut = 30 # 每轮运算超时设定

class workers(Process):
    def __init__(self, q, name, res):
        Process.__init__(self)
        self.q = q
        self.res = res
        self.name = name

    def run(self):
        print(self.name+" started!")
        while True:
            if not self.q.empty():
                try:
                    flie, virtualResult, cassingle, allCrazyFlies = self.q.get(False)
                    self.res.put(vorProcess(flie, virtualResult, cassingle, allCrazyFlies))
                except Exception as e:
                    pass

def multiProcess(taskPool, resultStorage):
    # 进程名称
    casadiLists = ["Process"+str(i) for i in range(1, 1+processNum)]
    # 储存列表
    processList = []

    for processName in casadiLists:
        # 创建新进程，传递输入和输出列表
        process = workers(taskPool, processName, resultStorage)
        # 将进程设置为守护进程，当主程序结束时，守护进程会被强行终止
        process.daemon = True
        process.start()
        processList.append(process)

    return processList

def vorProcess(flie, virtualResult, cassingle, allCrazyFlies):
    waypoints = []
    # 找出对应Id储存在allcrazyfiles中的索引
    [matchIndex] =  [index for (index, item) in enumerate(allCrazyFlies) if item['Id'] == flie['Id']]

    # 找到对应Id的虚拟维诺划分
    virtualFlie = [virtual for virtual in virtualResult if virtual['Id'] == flie['Id']][0]

    # casadi运算下一步位置
    outPut = cassingle.update(
        flie['vertices'],
        flie['centroid'],
        virtualFlie['vertices'],
        allCrazyFlies[matchIndex]['Position'],
        allCrazyFlies[matchIndex]['Pose']
    )

    # 待更新的位置信息
    newPosition = [pos for pos in outPut[-1][0:2]]
    newPose = round(outPut[-1][-1], 2)

    for timeIndex, item in enumerate(outPut):
        waypoints.append({
            'Id': allCrazyFlies[matchIndex]['Id'],
            'Px': item[0],
            'Py': item[1],
            'theta': item[2],
            'index': timeIndex
        })

    info = {
        "track": np.array(outPut),
        "Id": allCrazyFlies[matchIndex]['Id'],
        "newPosition": newPosition,
        "newPose": newPose,
        "waypoints": waypoints
    }

    return info

# 通过casadi计算得到结果
def getWaypoint():
    # 时间统计
    start = time.clock()

    vor = Vor(box, lineSpeed, angularSpeed)

    cassingle = Cassingle(lineSpeed, angularSpeed, T, N, xRange, yRange, volume, method="objective")

    if draw:
        graph = Graph([str(cf['Id']) for cf in allCrazyFlies], xRange, yRange)

    allWaypoints = []

    taskPool = Queue() # 根据参数创建队列

    resultStorage = Queue() # 储存进程运算结果的队列

    processList = multiProcess(taskPool, resultStorage)

    print("start calculating!")

    for counter in range(numIterations):
        print("epoch: {}, progress: {}%".format(
            counter,
            round(float(counter)/numIterations * 100, 2)
        ))

        # 更新维诺划分，下面过程中需要真实的和虚拟的位置
        vorResult = vor.updateVor(allCrazyFlies)
        virtualResult = vor.virtualVor(allCrazyFlies)

        # 将任务发布到队列中，等待守护进程进行处理
        for flie in vorResult:
            taskPool.put((flie, virtualResult, cassingle, allCrazyFlies))

        calculTime = time.clock()

        # 等待所有的任务执行完毕
        while True:
            # 超时，程序退出
            if time.clock() - calculTime > calculTimeOut:
                print("flies out of range, program exit!")
                sys.exit(0)

            if resultStorage.qsize() == len(allCrazyFlies):
                break

        waypoints = []

        # 将进程结果取出绘画出来
        while not resultStorage.empty():
            info = resultStorage.get()
            [matchIndex] =  [index for (index, item) in enumerate(allCrazyFlies) if item['Id'] == info["Id"]]
            allCrazyFlies[matchIndex]['Position'] = info['newPosition']
            allCrazyFlies[matchIndex]['Pose'] = info['newPose']
            draw and graph.updateTrack(
                    info['track'],
                    info["Id"]
                )
            waypoints += info['waypoints']

        # 根据时间索引进行排序
        waypoints = sorted(waypoints, key = lambda i: i['index'])

        allWaypoints.append(waypoints)

        # 更新维诺质心
        draw and graph.updateCentroid(
            np.array([cf['centroid'] for cf in vorResult]) # 真实位置维诺质心
            # np.array([cf['centroid'] for cf in virtualResult])
        )

        # 使用虚拟位置更新维诺边界
        draw and graph.updateRidges(virtualResult)
        # draw and graph.updateRidges(vorResult)

    print("consume: {}s to go through casadi".format(time.clock() - start))

    print("all children process closed.")

    return allWaypoints

if __name__ == "__main__":
    allWaypoints = []

    # --load从本地文件直接读取路径结果
    if args.load:
        f = open("record.txt", "rb")
        allWaypoints = pickle.load(f)
        f.close()
    else:
        allWaypoints = getWaypoint()

    # --record, 记录路径结果到本地txt文件，方便直接读取
    if args.record:
        f = open("record.txt", "wb")
        pickle.dump(allWaypoints, f)
        f.close()

    if not args.local:
        cfController = CFController(allCrazyFlies, N, T, Z, lineSpeed)
        print("casadi down, execute all waypoints")

        cfController.startFlies()
        for waypoints in allWaypoints:
            # 实时飞行
            cfController.goWaypoints(waypoints)

        # 降落
        cfController.goLand()