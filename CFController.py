# -*- coding: UTF-8 -*-
#!/usr/bin/env python
from pycrazyswarm import *
import numpy as np
import math

class CFController:
    def __init__(self, allCrazyFlies, N, T, Z, actualSpeed):
        self.executeNumber = 0
        self.actualSpeed = actualSpeed
        self.cfNum = len(allCrazyFlies)
        # height
        self.Z = Z
        self.framRate = float(N) / T
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
        self.allcfs.takeoff(targetHeight=self.Z, duration=1.0)
        # 等待2秒
        self.timeHelper.sleep(2.0)

    # 按照轨迹进行巡航
    def goWaypoints(self, waypoints):
        kPosition = 1.
        # 获取无人机字典
        allcfsDict = self.allcfs.crazyfliesById

        for waypoint in waypoints:
            # 取出实际位置和速度
            vx = self.actualSpeed * np.cos(waypoint['theta'])
            vy = self.actualSpeed * np.sin(waypoint['theta'])
            desiredPos = np.array([waypoint['Px'], waypoint['Py'], self.Z])

            # 获取对应ID的无人机控制器实例
            cf = allcfsDict[waypoint['Id']]

            error = desiredPos - cf.position()

            exec("yawRate = - round((waypoint['theta'] - self.lastYaw{}) / math.pi * 180 * self.framRate, 2)".format(waypoint['Id']))
            exec("self.lastYaw{} = waypoint['theta']".format(waypoint['Id']))

            # yawRate = (waypoint['theta'] - cf.yaw()) / math.pi * 180 * self.framRate
            cf.cmdVelocityWorld(np.array([vx, vy, 0] + kPosition * error), yawRate = yawRate)

            self.executeNumber += 1
            if(self.executeNumber == self.cfNum):
                self.timeHelper.sleepForRate(self.framRate)
                self.executeNumber = 0

    # 降落
    def goLand(self):
        print('Land!')
        allcfsDict = self.allcfs.crazyfliesById
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
                    self.timeHelper.sleepForRate(self.framRate)
                else:
                    cf.cmdStop()
                    cfs.remove(cf)
            if len(cfs)==0:
                    break