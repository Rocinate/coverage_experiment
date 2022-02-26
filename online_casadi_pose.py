# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import sys
import os
import yaml
import numpy as np
import time
time.clock = time.time
import math

# 添加路径
currentUrl = os.path.dirname(__file__)
parentUrl = os.path.abspath(os.path.join(currentUrl, os.pardir))
sys.path.append(parentUrl)

# 模拟参数 --local仅本地画图模拟
import argparse
parser = argparse.ArgumentParser()
parser.add_argument("--local", help="Run using local simulation.", action="store_true")
args = parser.parse_args()

if not args.local:
    # 无人机接口
    from pycrazyswarm import *
    from CFController import CFController

# 自定义库
from borderdVoronoi import Vor
from cassingle import Cassingle
from graphController import Graph

# 读取无人机位置配置
# with open("online_simulation/crazyfiles.yaml", "r") as f:
with open("crazyfiles.yaml", "r") as f:
    data = yaml.load(f)

allCrazyFlies = data['files']

STOP = False
numIterations = 50
xRange = 1.
yRange = 1.
box = np.array([-xRange, xRange, -yRange, yRange])  # 场地范围
lineSpeed = 0.05
angularSpeed = 0.5
draw =  True# 是否画图
T = 3.0
N = 6
allcfsTime = T/N
actualSpeed = 0.05
Z = 1.0 # 高度

if __name__ == "__main__":
    # 时间统计
    start = time.clock()

    vor = Vor(box, lineSpeed, angularSpeed)

    cassingle = Cassingle(lineSpeed, angularSpeed, T, N, xRange, yRange, method="objective")

    graph = Graph([str(cf['Id']) for cf in allCrazyFlies], xRange, yRange)

    if not args.local:
        cfController = CFController(allCrazyFlies, N, T, Z, actualSpeed)

    allWaypoints = []

    print("start calculating!")

    for counter in range(numIterations):
        print("epotch: {}, progress: {}%".format(
            counter,
            round(float(counter)/numIterations * 100, 2)
        ))

        # 更新维诺划分，下面过程中需要真实的和虚拟的位置
        vorResult = vor.updateVor(allCrazyFlies)
        virtualResult = vor.virtualVor(allCrazyFlies)

        waypoints = []
        for flie in vorResult:
            # 找出对应Id储存在allcrazyfiles中的索引
            [matchIndex] =  [index for (index, item) in enumerate(allCrazyFlies) if item['Id'] == flie['Id']]

            # 找到对应Id的虚拟维诺划分
            virtualFlie = [virtual for virtual in virtualResult if virtual['Id'] == flie['Id']][0]

            # casadi运算下一步位置
            outPut = cassingle.update(
                flie['vertices'],
                flie['centroid'],
                virtualFlie['vertices'],
                virtualFlie['centroid'],
                allCrazyFlies[matchIndex]['Position'],
                allCrazyFlies[matchIndex]['Pose']
            )

            allCrazyFlies[matchIndex]['Position'] = [pos for pos in outPut[-1][0:2]]
            allCrazyFlies[matchIndex]['Pose'] = round(outPut[-1][-1], 2)
            
            draw and graph.updateTrack(
                np.array(outPut), 
                allCrazyFlies[matchIndex]['Id']
            )

            for timeIndex, item in enumerate(outPut):
                waypoints.append({
                    'Id': allCrazyFlies[matchIndex]['Id'],
                    'Px': item[0],
                    'Py': item[1],
                    'theta': item[2],
                    'index': timeIndex
                })

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
    
    if not args.local:
        print("casadi down, execute all waypoints")
        cfController.startFlies()
        for waypoints in allWaypoints:
            # 实时飞行
            cfController.goWaypoints(waypoints)

        # 降落
        cfController.goLand()

    else:
        print("casadi down")

    print("consume: {}s to go through all program".format(time.clock() - start))