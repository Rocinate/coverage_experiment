# -*- coding: UTF-8 -*-
#!/usr/bin/env python
from pycrazyswarm import *
import numpy as np
import math
import json
from scipy.spatial.transform import Rotation

def print2txt(txt):
    with open("origin.txt", "w") as f:
        f.write(txt)


class CFController:
    def __init__(self, allCrazyFlies, N, T, Z, actualSpeed):
        self.executeNumber = 0
        self.actualSpeed = actualSpeed
        self.cfNum = len(allCrazyFlies)
        self.logBuffer = []
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

            actualPosition = cf.position()
            quaternion = cf.quaternion()

            rot = Rotation.from_quat(quaternion)
            actualPose = rot.as_euler("xyz")
            error = desiredPos - actualPosition

            self.logBuffer.append({
                "id": waypoint["Id"],
                "position": actualPosition.tolist()
            })

            # yawError = (waypoint['theta'] - actualPose[2]) / math.pi * 180
            # if (yawError >= 180):
            #     yawError  = 360 - yawError
            # elif(yawError <= -180):
            #     yawError += -yawError - 360

            # yawRate = - round(yawError, 2)

            exec("yawError = (waypoint['theta'] - self.lastYaw{}) / math.pi * 180".format(waypoint['Id']))

            if (yawError >= 180):
                yawError  = 360 - yawError
            elif(yawError <= -180):
                yawError += -yawError - 360

            exec("yawRate = (waypoint['theta'] - self.lastYaw{}) / math.pi * 180 * self.framRate".format(waypoint['Id']))

            exec("self.lastYaw{} = waypoint['theta']".format(waypoint['Id']))
            cf.cmdVelocityWorld(np.array([vx, vy, 0] + kPosition * error), yawRate = -yawError * self.framRate)
            #cf.cmdVelocityWorld(np.array([vx, vy, 0] + kPosition * error), yawRate = 0)

            self.executeNumber += 1
            if(self.executeNumber == self.cfNum):
                self.timeHelper.sleepForRate(self.framRate)
                self.executeNumber = 0

    # 降落
    def goLand(self):
        print('Land!')
        print2txt(json.dumps(self.logBuffer))
        print('saved data')
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