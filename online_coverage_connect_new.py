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
from multiprocessing import Process, Queue
from algorithms.connect_coverage_new.worker import Workers

# if python3
# time.clock = time.time

# 飞行参数
Z = 0.5 # 高度
dt = 0.1 # 控制器更新频率

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
# with open("online_simulation_dev/crazyfiles.yaml", "r") as f:
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
        _, ax = plt.subplots()
        # 动态绘图
        plt.ion()
        plt.title("UAVs track")

        n = len(allCrazyFlies)
        positions = np.zeros((n, 2))
        agentHandle = plt.scatter(positions[:, 0], positions[:, 1], marker=">", edgecolors="blue", c="white")

        count = 0
        
        # 初始化位置信息

        while not resultStorage.empty():
            waypoint = resultStorage.get()
            positions[count, 0] = waypoint['Px']
            positions[count, 1] = waypoint['Py']

            count += 1

            # 获取了所有无人机的位置信息，进行图像更新
            if count == n:
                count = 0
                agentHandle.set_offsets(positions)

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
            desiredPos = np.array([waypoint['Px'], waypoint['Py'], Z])

            # 获取对应ID的无人机控制器实例positions
            cf = allcfsDict[waypoint['Id']]

            actualPosition = cf.position()
            quaternion = cf.quaternion()

            rot = Rotation.from_quat(quaternion)
            actualPose = rot.as_euler("xyz")
            error = desiredPos - actualPosition

            cf.cmdVelocityWorld(np.array([vx, vy, 0] + kPosition * error), yawRate = 0)

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
