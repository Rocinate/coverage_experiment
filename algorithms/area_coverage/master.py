# -*- coding: UTF-8 -*-
#!/usr/bin/env python
from multiprocessing import Process, Queue
from scipy.spatial.transform import Rotation
import numpy as np
import traceback  # 错误堆栈


class Master(Process):
    def __init__(self, name, res: Queue, graphPipeLine: Queue, allCrazyFlies, dt, Z, kPosition, epochNum, allcfs=None, timeHelper=None):
        Process.__init__(self)
        self.epoch = 0
        self.epochNum = epochNum
        self.name = name
        self.res = res
        self.allCrazyFlies = allCrazyFlies
        self.dt = dt
        self.graphPipeLine = graphPipeLine  # 用于将数据返回给主线程进行绘画
        self.Z = Z  # 飞行高度
        # 修正系数
        self.kPosition = kPosition
        self.allcfs = allcfs
        self.timeHelper = timeHelper
        self.framRate = 1.0 / self.dt
        self.publish = True if allcfs != None else False

    def init(self):
        # 所有无人机同时起飞
        self.allcfs.takeoff(targetHeight=self.Z, duration=1.0)
        # 等待2秒
        self.timeHelper.sleep(2.0)

        # 获取无人机字典
        self.allcfsDict = self.allcfs.crazyfliesById

    def run(self):
        print("Master server started")

        try:
            executeNumber = 0

            # 位置信息暂存，收集完毕后通过graphPipeLine发送给绘图线程
            n = len(self.allCrazyFlies)

            positions = np.zeros((n, 2))

            # 起飞✈
            # self.publish and self.init()

            while self.epoch < self.epochNum - 1:
                if not self.res.empty():
                    waypoint = self.res.get()
                    # 取出实际位置和速度
                    vx = waypoint['ux']
                    vy = waypoint['uy']
                    vz = waypoint['uz']

                    # 信息储存
                    positions[executeNumber, 0] = waypoint['Px']
                    positions[executeNumber, 1] = waypoint['Py']
                    executeNumber += 1

                    # 指令广播
                    if self.publish:
                        # 获取对应ID的无人机控制器实例positions
                        cf = self.allcfsDict[waypoint['Id']]

                        quaternion = cf.quaternion()

                        rot = Rotation.from_quat(quaternion)
                        # 四元数进行结算后的真实角度
                        actualPose = rot.as_euler("xyz")

                        actualPosition = cf.position()
                        # 正常飞行
                        if vz == 0:
                            desiredPos = np.array([waypoint['Px'], waypoint['Py'], self.Z])
                            error = desiredPos - actualPosition
                            cf.cmdVelocityWorld(
                                np.array([vx, vy, vz] + self.kPosition * error), yawRate=0)
                        # 损坏坠落
                        elif actualPosition[-1] > 0.05:
                            desiredPos = np.array(
                                [waypoint['Px'], waypoint['Py'], actualPosition[-1] + self.dt * vz])
                            error = desiredPos - actualPosition
                            cf.cmdVelocityWorld(
                                np.array([vx, vy, vz] + self.kPosition * error), yawRate=0)
                        else:
                            cf.cmdStop()

                    if executeNumber == n:
                        self.publish and self.timeHelper.sleepForRate(self.framRate)
                        self.epoch += 1
                        executeNumber = 0

                        # 返回位置信息，用于绘图
                        self.graphPipeLine.put(positions)

            # 执行完毕，停止飞机
            self.publish and self.stop()
        except Exception as e:
            print(traceback.print_exc()) # debug exception

    def stop(self):
        print('Land!')
        cfs = self.allcfsDict.values()
        i = 0
        while True:
            i = i+1
            for cf in cfs:
                current_pos = cf.position()
                if current_pos[-1] > 0.05:
                    vx = 0
                    vy = 0
                    vz = -0.3
                    cf.cmdVelocityWorld(np.array([vx, vy, vz]), yawRate=0)
                    self.timeHelper.sleepForRate(self.framRate)
                else:
                    cf.cmdStop()
                    cfs.remove(cf)
            if len(cfs) == 0:
                break
