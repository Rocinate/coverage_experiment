#!/usr/bin/env python

import numpy as np
import os
import sys
import argparse
# 添加路径
currentUrl = os.path.dirname(__file__)
parentUrl = os.path.abspath(os.path.join(currentUrl, os.pardir))
sys.path.append(parentUrl)
from pycrazyswarm import *

# 模拟参数 --local仅本地画图模拟
parser = argparse.ArgumentParser()
parser.add_argument("--mode", type=int, help="Run using middle control.", required=True, default=1)
args = parser.parse_args()

class Waypoint:
    def __init__(self, agent, vx, vy, x, y, z, arrival, duration):
        self.agent = agent
        self.vx = vx
        self.vy = vy
        self.x = x
        self.y = y
        self.z = z
        self.arrival = arrival
        self.duration = duration

    def __lt__(self, other):
        return self.arrival < other.arrival

    def __repr__(self):
        return "Ag {} at {} s. [{}, {}, {}]".format(self.agent, self.arrival, self.x, self.y, self.z)


if __name__ == "__main__":
    # load csv file
    data = np.loadtxt("newWaypoints.csv", skiprows=1, delimiter=',')

    # sort by agents
    data[data[:,0].argsort()]

    # convert to internal data structure
    waypoints = []
    lastAgent = None
    for row in data:
        if lastAgent is None or lastAgent != row[0]:
            lastTime = 0.0
        waypoints.append(Waypoint(
            int(row[0]),
            row[1],
            row[2],
            row[3],
            row[4],
            row[4] - lastTime))
        lastTime = row[4]
        lastAgent = int(row[0])

    # sort waypoints by arrival time
    waypoints.sort()

    # execute waypoints
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # 所有无人同时起飞
    allcfs.takeoff(targetHeight=0.6, duration=2.0)
    # 等待休眠2轮
    timeHelper.sleep(2.0)
    lastTime = 0.0
    for waypoint in waypoints:
        if waypoint.arrival == 0:
            pos = [waypoint.x, waypoint.y, waypoint.z]
            # print(waypoint.agent, pos, 2.0)
            cf = allcfs.crazyfliesById[waypoint.agent]
            cf.goTo(pos, 0, 2.0)
        elif waypoint.duration > 0:
            timeHelper.sleep(waypoint.arrival - lastTime)
            lastTime = waypoint.arrival
            pos = [waypoint.x, waypoint.y, waypoint.z]
            # print(waypoint.agent, pos, waypoint.duration)
            cf = allcfs.crazyfliesById[waypoint.agent]
            cf.goTo(pos, 0, waypoint.duration)

    # land
    allcfs.land(targetHeight=0.02, duration=2.0)
    timeHelper.sleep(2.0)
