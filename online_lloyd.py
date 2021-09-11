# -*- coding: UTF-8 -*-
#!/usr/bin/env python
from pycrazyswarm import *
import signal
import numpy as np
import scipy.spatial as sp
import sys
import matplotlib.path as mpltPath

eps = sys.float_info.epsilon

# 初始化位置信息
Position = {
    '2': [0.5, 0.9],
    '4': [-0.5, 0.9],
    '5': [0.5, -0.9],
    '7': [-0.5, -0.9],
}


STOP = False          # 中止标志位
numIterations = 3   # 迭代次数
xRange = 0.6            # 场地长度
yRange = 1.0            # 场地宽度
box = np.array([-xRange, xRange, -yRange, yRange])  # 场地范围
speed =  0.5               

# 判断点是否在场地范围内
def in_box(towers, bounding_box):
    return np.logical_and(np.logical_and(bounding_box[0] <= towers[:, 0],
                                         towers[:, 0] <= bounding_box[1]),
                          np.logical_and(bounding_box[2] <= towers[:, 1],
                                         towers[:, 1] <= bounding_box[3]))

# 利用对称性质，生成带有边界点的维诺划分
def voronoi(towers, bounding_box):
    # 选取在范围内的点
    i = in_box(towers, bounding_box)
    # 对范围内的点按照边界进行镜像
    points_center = towers[i, :]
    points_left = np.copy(points_center)
    points_left[:, 0] = bounding_box[0] - (points_left[:, 0] - bounding_box[0])
    points_right = np.copy(points_center)
    points_right[:, 0] = bounding_box[1] + (bounding_box[1] - points_right[:, 0])
    points_down = np.copy(points_center)
    points_down[:, 1] = bounding_box[2] - (points_down[:, 1] - bounding_box[2])
    points_up = np.copy(points_center)
    points_up[:, 1] = bounding_box[3] + (bounding_box[3] - points_up[:, 1])
    points = np.append(points_center,
                       np.append(np.append(points_left,
                                           points_right,
                                           axis=0),
                                 np.append(points_down,
                                           points_up,
                                           axis=0),
                                 axis=0),
                       axis=0)
    # 计算维诺划分
    vor = sp.Voronoi(points)
    # 过滤无限划分区域
    regions = []
    for region in vor.regions:
        flag = True
        for index in region:
            if index == -1:
                flag = False
                break
            else:
                x = vor.vertices[index, 0]
                y = vor.vertices[index, 1]
                if not(bounding_box[0] - eps <= x and x <= bounding_box[1] + eps and
                       bounding_box[2] - eps <= y and y <= bounding_box[3] + eps):
                    flag = False
                    break
        if region != [] and flag:
            regions.append(region)
    vor.filtered_points = points_center
    vor.filtered_regions = regions
    return vor

# 维诺质心计算
def centroid_region(vertices):
    # Polygon's signed area
    A = 0
    # Centroid's x
    C_x = 0
    # Centroid's y
    C_y = 0
    for i in range(0, len(vertices) - 1):
        s = (vertices[i, 0] * vertices[i + 1, 1] - vertices[i + 1, 0] * vertices[i, 1])
        A = A + s
        C_x = C_x + (vertices[i, 0] + vertices[i + 1, 0]) * s
        C_y = C_y + (vertices[i, 1] + vertices[i + 1, 1]) * s
    A = 0.5 * A
    C_x = (1.0 / (6.0 * A)) * C_x
    C_y = (1.0 / (6.0 * A)) * C_y
    return np.array([[C_x, C_y]])

# 以固定速度计算所有无人机飞到标准点所需要的时间
def getTime(positions, centroids):
    maxTime = 0.0
    for index, item in enumerate(positions):
        distance = np.linalg.norm(np.array(item) - np.array(centroids[index]))
        time = distance / speed
        maxTime = maxTime if(maxTime > time) else time
    return maxTime


# 匹配维诺区域与无人机，返回无人机位置序列
def matchPoint(vor):
    waypoints = []
    centroids = []
    posList = np.array([Position[key] for key in Position])
    Id = list(Position.keys())
    # 对维诺区域进行遍历
    for region in vor.filtered_regions:
        vertices = vor.vertices[region + [region[0]], :]
        # 计算质心
        centroid = centroid_region(vertices)
        centroids.append(centroid[0, :].tolist())
        # 判断点是否在区域内
        path = mpltPath.Path(vertices)
        # inside2 = posList[path.contains_points(posList)][0,:]
        index = np.where(path.contains_points(posList))[0][0]
        # print(vertices, posList, index)
        # print(vertices, posList, index, Id[index], centroid[0, :])
        Position[Id[index]] = centroid[0, :].tolist()
    allcfsTime = getTime(posList.tolist(), centroids)

    for key in Position.keys():
        waypoints.append(Waypoint(
                int(key),
                Position[key][0],
                Position[key][1],
                allcfsTime
            ))
    return waypoints


# 中断处理函数，防止不降落
def stop_handler(signum, frame):
    global STOP
    STOP = True
    print("进程被终止，准备降落")

# 位置类
class Waypoint:
    def __init__(self, agent, x, y, duration, z = 0.5):
        self.agent = agent
        self.x = x
        self.y = y
        self.z = z
        self.duration = duration

    # 这个比较规则，在每轮都生成新的目标点的情况下没有必要使用
    # def __lt__(self, other):
    #     return self.arrival < other.arrival

    def __repr__(self):
        return "Ag {}. [{}, {}, {}]".format(self.agent, self.x, self.y, self.z)


if __name__ == "__main__":
    # 设置相应信号处理的handler
    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    # 创建无人机实例
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    # 所有无人机同时起飞
    allcfs.takeoff(targetHeight=0.6, duration=2.0)
    # 等待2秒
    timeHelper.sleep(2.0)

    # 进行维诺过程
    for counter in range(0, numIterations):
        # 维诺划分
        vor = voronoi(np.array(
            [Position[key] for key in Position]
        ), box)

        # 根据计算出来的维诺质心进行无人机匹配
        waypoints = matchPoint(vor)

        # 根据到达时间排序，在本同步任务中无需使用
        # waypoints.sort()

        # 遍历无人机，发送指令
        for waypoint in waypoints:
            pos = [waypoint.x, waypoint.y, waypoint.z]
            print(waypoint.agent, pos)
            cf = allcfs.crazyfliesById[waypoint.agent]
            cf.goTo(pos, 0, waypoint.duration)
        
        # 通过ros定时器去休眠
        timeHelper.sleep(waypoints[0].duration)
        
        while True:
            arrived = 0
            for waypoint in waypoints:
                cf = allcfs.crazyfliesById[waypoint.agent]
                error = np.linalg.norm(np.array(Position[waypoint.agent]) - cf.position()[0:2])
                if(error < 0.1): 
                    arrived += 1
                    # 更新下一轮迭代所需要的位置信息
                    Position[waypoint.agent] = cf.position[0:2].tolist()

            # 全部无人机到位，退出循环
            if(arrived == len(Position.keys())):
                print("iteration {} pass.".format(counter))
                break

        # 收到终止信号，停止运行
        if(STOP):
            # 原地降落
            allcfs.land(targetHeight=0.02, duration=2.0)
            timeHelper.sleep(2.0)
            break
    
    # 执行完毕，降落
    allcfs.land(targetHeight=0.02, duration=2.0)
    timeHelper.sleep(2.0)
