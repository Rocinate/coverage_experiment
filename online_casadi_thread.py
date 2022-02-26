# -*- coding: UTF-8 -*-
#!/usr/bin/env python
# 添加环境变量
import sys
import os
currentUrl = os.path.dirname(__file__)
parentUrl = os.path.abspath(os.path.join(currentUrl, os.pardir))
sys.path.append(parentUrl)

# 添加依赖
import yaml
import numpy as np
import time
import datetime
import threading
from CFController import CFController
from borderdVoronoi import Vor
from cassingle import Cassingle
from graphController import Graph

# 读取无人机位置配置
with open("online_simulation/crazyfiles.yaml", "r") as f:
    data = yaml.load(f)

allCrazyFlies = data['files']

STOP = False
numIterations = 30
xRange =0.7
yRange =0.95
box = np.array([-xRange, xRange, -yRange, yRange])  # 场地范围
lineSpeed = 0.05
angularSpeed = 0.5
T = 3.0
N = 6
Z = 0.5
draw = False # 是否画图
allcfsTime = T/N
actualSpeed = 0.05

allWaypoints = []
allLoss = []
# thread2 alive?
status = 1

def calcul(numIterations):
    global status
    vor = Vor(box, lineSpeed, angularSpeed)

    cassingle = Cassingle(lineSpeed, angularSpeed, T, N, xRange, yRange)
    
    if(draw):
        graph = Graph([str(cf['Id']) for cf in allCrazyFlies], xRange, yRange)

    for counter in range(numIterations):
        print("epoch: {}, progress: {}%".format(
            counter,
            round(float(counter)/numIterations * 100, 2)
        ))
        # 更新维诺划分
        vorResult = vor.updateVor(allCrazyFlies)

        waypoints = []

        for flie in vorResult:
            # 找出对应Id储存在allcrazyfiles中的索引
            [matchIndex] =  [index for (index, item) in enumerate(allCrazyFlies) if item['Id'] == flie['Id']]
            # casadi运算下一步位置
            outPut = cassingle.update(
                flie['vertices'], 
                flie['centroid'], 
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
            np.array([cf['centroid'] for cf in vorResult])
        )
        
        # 使用虚拟位置更新维诺边界
        virtualResult = vor.virtualVor(allCrazyFlies)
        draw and graph.updateRidges(virtualResult)


        # 计算loss
        virtualPosition = vor.virtualPosition(allCrazyFlies)
        loss = cassingle.loss(np.array(
            [cf['Position'] for cf in virtualPosition]
        ))
        
        # 储存
        allLoss.append(loss)
        while len(allWaypoints) > 3:
            time.sleep(0.1)
    print('calcul done')
    status = 0

if __name__ == "__main__":
    # 时间统计
    start = datetime.datetime.now()

    # 创建飞控实例
    cfController = CFController(allCrazyFlies, N, T, Z, actualSpeed)

    # 创建副线程并启动
    thread2 = threading.Thread(target=calcul, args=(numIterations,))
    thread2.start()

    # 开飞🎉
    cfController.startFlies()
    # 查看无人机状态
    while status or len(allWaypoints) > 0:
        # 从缓冲区取数据
        if(len(allWaypoints) > 0):
            cfController.goWaypoints(allWaypoints.pop(0))
        else:
            time.sleep(0.1)
    
    cfController.goLand()

    end = datetime.datetime.now()
    print("time consumed: {}s".format(end-start))

    print('two thread end')