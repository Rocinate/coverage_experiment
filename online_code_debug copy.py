# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import sys

import os

currentUrl = os.path.dirname(__file__)
parentUrl = os.path.abspath(os.path.join(currentUrl, os.pardir))
sys.path.append(parentUrl)

from pycrazyswarm import *
import yaml
import numpy as np
from borderdVoronoi import Vor
from cassingle import Cassingle
from graphController import Graph
import time
from tqdm.notebook import trange
import math

# 读取无人机位置配置
with open("online_simulation/crazyfiles.yaml", "r") as f:
    data = yaml.load(f)

allCrazyFlies = data['files']

STOP = False
numIterations = 5
xRange = 1.
yRange = 1.
box = np.array([-xRange, xRange, -yRange, yRange])  # 场地范围
lineSpeed = 0.05
angularSpeed = 0.5
T = 3.0
N = 6
draw =  False# 是否画图
allcfsTime = T/N
actualSpeed = 0.05
Z = 1.0 # 高度

class CFController():
    def __init__(self):
        self.executeNumber = 0
        names = self.__dict__

        for cf in allCrazyFlies:
            names['lastYaw'+str(cf['Id'])] = 0.0

    def startFlies(self):
        print('Start flying!')

        # 创建无人机实例
        swarm = Crazyswarm()
        self.timeHelper = swarm.timeHelper
        self.allcfs = swarm.allcfs

        # 所有无人机同时起飞
        self.allcfs.takeoff(targetHeight=Z, duration=1.0)
        # 等待2秒
        self.timeHelper.sleep(2.0)

    # 按照轨迹进行巡航
    def goWaypoints(self, waypoints):
        kPosition = 1.
        # 获取无人机字典
        allcfsDict = self.allcfs.crazyfliesById

        for waypoint in waypoints:
            # 取出实际位置和速度
            vx = actualSpeed * np.cos(waypoint['theta'])
            vy = actualSpeed * np.sin(waypoint['theta'])
            desiredPos = np.array([waypoint['Px'], waypoint['Py'], Z])

            # 获取对应ID的无人机控制器实例
            cf = allcfsDict[waypoint['Id']]

            error = desiredPos - cf.position()

            exec("yawRate = round((waypoint['theta'] - self.lastYaw{}) / math.pi * 180, 2)".format(waypoint['Id']))
            exec("self.lastYaw{} = waypoint['theta']".format(waypoint['Id']))

            cf.cmdVelocityWorld(np.array([vx, vy, 0] + kPosition * error), yawRate=yawRate)

            self.executeNumber += 1
            if(self.executeNumber == len(allCrazyFlies)):
                self.timeHelper.sleepForRate(float(N)/T)
                self.executeNumber = 0

    # 降落
    def goLand(self):
        print('Land!')
        allcfsDict = self.allcfs.crazyfliesById
        cfs = allcfsDict.values()
        i = 0
        startTime = self.timeHelper.time()
        while True:
            i=i+1
            for cf in cfs:
                current_pos=cf.position()
                if current_pos[-1]>0.05:
                    vx=0
                    vy=0
                    vz=-0.1
                    cf.cmdVelocityWorld(np.array([vx, vy, vz] ), yawRate=0)
                    self.timeHelper.sleepForRate(float(N)/T)
                else:
                    cf.cmdStop()
                    cfs.remove(cf)
            if len(cfs)==0:
                    break
        endTime=self.timeHelper.time()
        print('Total loop number of landing is '+str(i))
        print('Total land time is '+str(endTime-startTime))



if __name__ == "__main__":
    # 时间统计
    start = time.clock()

    vor = Vor(box, lineSpeed, angularSpeed)

    cassingle = Cassingle(lineSpeed, angularSpeed, T, N, xRange, yRange, method="objective")

    graph = Graph([str(cf['Id']) for cf in allCrazyFlies], xRange, yRange)

    cfController = CFController()

    allWaypoints = []

    print("start calculating!")

    for counter in trange(numIterations):
        # 更新维诺划分，下面过程中需要真实的和虚拟的位置
        vorResult = vor.updateVor(allCrazyFlies)
        virtualResult = vor.virtualVor(allCrazyFlies)

        waypoints = []
        for flie in vorResult:
            # 找出对应Id储存在allcrazyfiles中的索引
            [matchIndex] =  [index for (index, item) in enumerate(allCrazyFlies) if item['Id'] == flie['Id']]

            # 找到对应Id的虚拟维诺划分
            virtualFlie = [virtual for virtual in virtualResult if virtual['Id'] == flie['Id']]

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
    
    print("casadi down, execute all waypoints")

    cfController.startFlies()
    for waypoints in allWaypoints:
        # 实时飞行
        cfController.goWaypoints(waypoints)

    # 降落
    cfController.goLand()

    print("couse: {}s to go through all program".format(time.clock() - start))