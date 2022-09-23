# -*- coding: UTF-8 -*-
#!/usr/bin/env python
from multiprocessing import Process, Queue
import numpy as np
from scipy.spatial.transform import Rotation
import traceback # 错误堆栈

class Master(Process):
    def __init__(self, name, res: Queue, graphPipeLine: Queue, allCrazyFlies, dt, Z, kPosition, allcfs=None, timeHelper=None):
        Process.__init__(self)
        self.name = name
        self.res = res
        self.allCrazyFlies = allCrazyFlies
        self.dt = dt
        self.graphPipeLine = graphPipeLine # 用于将数据返回给主线程进行绘画
        self.publish = publish # 消息发布标志位，是否进行控制
        self.Z = Z # 飞行高度
        # 修正系数
        self.kPosition = kPosition

    def init(self):
        # 所有无人机同时起飞
        self.allcfs.takeoff(targetHeight=self.Z, duration=1.0)
        # 等待2秒
        self.timeHelper.sleep(2.0)

        # 获取无人机字典
        self.allcfsDict = self.allcfs.crazyfliesById

    def run(self):
        print("Master server started")

        framRate = 1.0 / self.dt

        print('Start flying!')

        executeNumber = 0

        while not self.res.empty():
            waypoint = self.res.get()
            # 取出实际位置和速度
            vx = waypoint['ux']
            vy = waypoint['uy']
            vz = waypoint['uz']

            # 获取对应ID的无人机控制器实例positions
            cf = allcfsDict[waypoint['Id']]

            quaternion = cf.quaternion()

            rot = Rotation.from_quat(quaternion)
            actualPose = rot.as_euler("xyz")

            actualPosition = cf.position()
            # 正常飞行
            if vz == 0:
                desiredPos = np.array([waypoint['Px'], waypoint['Py'], Z])
                error = desiredPos - actualPosition
                cf.cmdVelocityWorld(np.array([vx, vy, vz] + kPosition * error), yawRate = 0)
            # 损坏坠落
            elif actualPosition[-1] > 0.05:
                desiredPos = np.array([waypoint['Px'], waypoint['Py'], actualPosition[-1] + dt * vz])
                error = desiredPos - actualPosition
                cf.cmdVelocityWorld(np.array([vx, vy, vz] + kPosition * error), yawRate = 0)
            else:
                cf.cmdStop()

            executeNumber += 1
            if(executeNumber == len(self.allCrazyFlies)):
                timeHelper.sleepForRate(framRate)
                executeNumber = 0

        print('Land!')
        # print2txt(json.dumps(self.logBuffer))
        # print('saved data')
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
                    timeHelper.sleepForRate(framRate)
                else:
                    cf.cmdStop()
                    cfs.remove(cf)
            if len(cfs)==0:
                    break

        # try:
        #     while True:
        #         if self.res.not_empty():
