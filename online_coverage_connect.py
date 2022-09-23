# -*- coding: UTF-8 -*-
#!/usr/bin/env python
from scipy.spatial.transform import Rotation
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
from algorithms.connect_coverage.worker import Workers

# if python3
# time.clock = time.time

# 飞行参数
Z = 0.5 # 高度
dt = 0.1 # 控制器更新频率

np.random.seed(42)

# 参数配置
r = 2.0 # 雷达半径
circleX, circleY = 6.0, 0.  # 雷达中心
angleStart, angleEnd = np.pi*165/180, np.pi*195/180  # 扇面覆盖范围30°
cov = 5/180*np.pi  # 单机覆盖角度
# 覆盖范围
positionStart = -2.5
positionEnd = 3.0

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
with open("crazyfiles.yaml", "r") as f:
    data = yaml.load(f, Loader=yaml.FullLoader)
allCrazyFlies = data['files']

# 实验参数
STOP = False

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
        process = Workers('Process1', resultStorage, allCrazyFlies, dt)
        # 将进程设置为守护进程，当主程序结束时，守护进程会被强行终止
        process.daemon = True
        process.start()

    # --record, 记录路径结果到本地txt文件，方便直接读取
    if args.record:
        f = open("record.txt", "wb")
        pickle.dump(allWaypoints, f)
        f.close()

    if args.local:
        _, ax = plt.subplots(figsize=(8,12))

        intNum = 20  # 覆盖扇面插值数
        angleList = np.linspace(angleStart, angleEnd, intNum)  # 计算覆盖扇面位置,用于作图
        # 扇形点位，添加起点保证图像闭合
        # xList = [circleX] + [circleX + r *
        #                     np.cos(angle) for angle in angleList] + [circleX]
        # yList = [circleY] + [circleY + r *
        #                     np.sin(angle) for angle in angleList] + [circleY]

        epoch = 0
        # 动态绘图
        plt.ion()
        titleHandle = plt.title("UAVs track epoch " + str(epoch))
        plt.xlim([-5, 10])
        plt.ylim([-15, 15])

        n = len(allCrazyFlies)
        positions = np.zeros((n*3, 2))
        agentHandle = plt.scatter(positions[:, 0], positions[:, 1], marker=">", edgecolors="blue", c="white")
        angles = np.array([np.pi for _ in range(n*3)])

        diff1 = np.random.rand(n, 2)
        diff1[:, 0] = diff1[:, 0] / 2
        diff2 = np.random.rand(n ,2)
        diff2[:, 0] = diff2[:, 0] / 2
        # 覆盖扇面作图
        verHandle = [None] * n * 3
        for index in range(n):
            # 初始化
            patch = patches.Polygon([
                [circleX + r * np.cos(np.pi-cov/2), circleY + r * np.sin(angles[index]-cov/2)],
                [circleX + r * np.cos(np.pi+cov/2), circleY + r * np.sin(angles[index]+cov/2)],
                [circleX, circleY]
            ], fill=False)
            verHandle[index] = ax.add_patch(patch)

        for index in range(n, 2*n):
            # 初始化
            patch = patches.Polygon([
                [circleX + r * np.cos(np.pi-cov/2), circleY  + 10+ r * np.sin(angles[index]-cov/2)],
                [circleX + r * np.cos(np.pi+cov/2), circleY + 10+ r * np.sin(angles[index]+cov/2)],
                [circleX, circleY]
            ], fill=False)
            verHandle[index] = ax.add_patch(patch)

        for index in range(2*n, 3*n):
            # 初始化
            patch = patches.Polygon([
                [circleX + r * np.cos(np.pi-cov/2), circleY -10  + r * np.sin(angles[index]-cov/2)],
                [circleX + r * np.cos(np.pi+cov/2), circleY -10  + r * np.sin(angles[index]+cov/2)],
                [circleX, circleY]
            ], fill=False)
            verHandle[index] = ax.add_patch(patch)

        plt.show()
        count = 0
        
        # 初始化位置信息
        while not resultStorage.empty():
            waypoint = resultStorage.get()
            positions[count, 0] = waypoint['Px']
            positions[count, 1] = waypoint['Py']
            angles[count] = waypoint['theta']
            count += 1

            # 获取了所有无人机的位置信息，进行图像更新
            if count == n:
                epoch += 1
                count = 0
                
                positions[n:2*n, :] = positions[0:n, :] + diff1
                positions[n:2*n, 1] = positions[n:2*n, 1] + 10 

                positions[2*n:3*n, :] = positions[0:n, :] + diff2
                positions[2*n:3*n, 1] = positions[2*n:3*n, 1] - 10

                for flightIndex in range(n, 2*n):
                    # if flightIndex % 2 == 0:
                    positions[flightIndex, 0] = positions[flightIndex, 0]
                    positions[flightIndex, 1] = 20 - positions[flightIndex, 1]
                    # else:
                    #     positions[flightIndex, 0] = positions[flightIndex, 0] - 1
                    #     positions[flightIndex, 1] = positions[flightIndex, 1] 
                
                for flightIndex in range(2*n, 3*n):
                    positions[flightIndex, 0] = -16/121 * positions[flightIndex, 0]**2 + 8/121 * positions[flightIndex, 0] + 1 + positions[flightIndex, 0]
                    positions[flightIndex, 1] = positions[flightIndex, 1]

                angles[n:2*n] = np.pi + np.arctan((circleY +10 - positions[n:2*n, 1]) / (circleX - positions[n:2*n, 0]))
                angles[2*n:3*n] = np.pi + np.arctan((circleY - 10 - positions[2*n:3*n, 1]) / (circleX - positions[2*n:3*n, 0]))

                agentHandle.set_offsets(positions)
                plt.setp(titleHandle, text = "UAVs track epoch "+str(epoch))

                for idx, angle in enumerate(angles):
                    if angle < angleEnd and angle > angleStart and idx < n:
                        path = [
                            [circleX + r * np.cos(angle - cov/2), circleY + r * np.sin(angle - cov/2)],
                            [circleX + r * np.cos(angle + cov/2), circleY + r * np.sin(angle + cov/2)],
                            [circleX, circleY]
                        ]
                        plt.setp(verHandle[idx], xy=path)
                    if angle < angleEnd and angle > angleStart and idx >= n and idx < 2*n:
                        path = [
                            [circleX + r * np.cos(angle - cov/2), circleY +10+ r * np.sin(angle - cov/2)],
                            [circleX + r * np.cos(angle + cov/2), circleY +10+ r * np.sin(angle + cov/2)],
                            [circleX, circleY+10]
                        ]
                        plt.setp(verHandle[idx], xy=path)
                    if angle < angleEnd and angle > angleStart and idx >= 2*n and idx < 3*n:
                        path = [
                            [circleX + r * np.cos(angle - cov/2), circleY -10+ r * np.sin(angle - cov/2)],
                            [circleX + r * np.cos(angle + cov/2), circleY -10+ r * np.sin(angle + cov/2)],
                            [circleX, circleY-10]
                        ]
                        plt.setp(verHandle[idx], xy=path)
                plt.pause(0.000000000001)
        plt.ioff()
        plt.show()

    if not args.local:
        framRate = 1.0 / dt

        print('Start flying!')

        # 创建无人机实例
        swarm = Crazyswarm()
        timeHelper = swarm.timeHelper
        allcfs = swarm.allcfs

        # 所有无人机同时起飞
        allcfs.takeoff(targetHeight=Z, duration=1.0)
        # 等待2秒
        timeHelper.sleep(2.0)

        # 修正系数
        kPosition = 1.
        # 获取无人机字典
        allcfsDict = allcfs.crazyfliesById

        executeNumber = 0

        while not resultStorage.empty():
            waypoint = resultStorage.get()
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
            if(executeNumber == len(allCrazyFlies)):
                timeHelper.sleepForRate(framRate)
                executeNumber = 0

        print('Land!')
        # print2txt(json.dumps(self.logBuffer))
        # print('saved data')
        allcfsDict = allcfs.crazyfliesById
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
