# -*- coding: UTF-8 -*-
#!/usr/bin/env pythonW
import signal
import numpy as np
import scipy.spatial as sp
import sys
import matplotlib.path as mpltPath
import casadi as csd
import matplotlib.pyplot as plt
import matlab.engine

eps = sys.float_info.epsilon

# 初始化位置信息
Position = {
    '2': [-0.3, 0.45],
    '4': [-0.1, 0.15],
    '5': [-0.3, -0.15],
    '7': [-0.1, -0.45],
    '9': [-0.1, -0.35],
}
# 无人机姿态数据
Pose = {
    '2': 0.0,
    '4': 0.0,
    '5': 0.0,
    '7': 0.0,
    '9': 0.0,
}

# 无人机历史控制量
u_his = {
    '2': 0.0,
    '4': 0.0,
    '5': 0.0,
    '7': 0.0,
    '9': 0.0,
}

# 无人机轨迹储存
track_his = {
    '2': [[], []],
    '4': [[], []],
    '5': [[], []],
    '7': [[], []],
    '9': [[], []],
}


STOP = False          # 中止标志位
numIterations = 50   # 迭代次数
xRange = 0.5           # 场地长度
yRange = 0.7            # 场地宽度
box = np.array([-xRange, xRange, -yRange, yRange])  # 场地范围
vv = 0.05    # 速度
ww = 0.5   # 角度
T = 3.0
N = 6.0
draw = True  # 是否画图
allcfsTime = T/N

# 创建绘图句柄
if(draw):
    fig, ax = plt.subplots()
    # 根据无人机ID动态申请变量
    names = locals()
    for i in Position.keys():
        # 加逗号，不然取不出来
        names['line' + i], = plt.plot([], [], '.-', label='flight'+i)
        names['track_his[' + i + ']'] = [[], []]
    plt.legend()
    for i in Position.keys():
        names['ridges_handle' + i], = plt.plot([], [], 'k-', label='ridges')
    centroids_handle, = plt.plot([], [], 'go', label='centriods')
    
    # 规定画图范围
    ax.set_xlim([-xRange, xRange])
    ax.set_ylim([-yRange, yRange])

# 判断点是否在场地范围内


def in_box(towers, bounding_box):
    return np.logical_and(np.logical_and(bounding_box[0] <= towers[:, 0],
                                         towers[:, 0] <= bounding_box[1]),
                          np.logical_and(bounding_box[2] <= towers[:, 1],
                                         towers[:, 1] <= bounding_box[3]))

# 利用对称性质，生成带有边界点的维诺划分


def voronoi(towers, bounding_box):
    # print('towers: ', towers)
    # 选取在范围内的点
    i = in_box(towers, bounding_box)
    # print(i)
    # 对范围内的点按照边界进行镜像
    points_center = towers[i, :]
    points_left = np.copy(points_center)
    points_left[:, 0] = bounding_box[0] - (points_left[:, 0] - bounding_box[0])
    points_right = np.copy(points_center)
    points_right[:, 0] = bounding_box[1] + \
        (bounding_box[1] - points_right[:, 0])
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
                x = round(vor.vertices[index, 0], 2)
                y = round(vor.vertices[index, 1], 2)
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
        s = (vertices[i, 0] * vertices[i + 1, 1] -
             vertices[i + 1, 0] * vertices[i, 1])
        A = A + s
        C_x = C_x + (vertices[i, 0] + vertices[i + 1, 0]) * s
        C_y = C_y + (vertices[i, 1] + vertices[i + 1, 1]) * s
    A = 0.5 * A
    C_x = (1.0 / (6.0 * A)) * C_x
    C_y = (1.0 / (6.0 * A)) * C_y
    return [C_x, C_y]


# 卡萨帝求解
# vertices： numpy
def cassingle(vertices, Id):
    # 通过matlab调用自定义casadi方法
    print(vertices.shape, '11')
    x_opt = eng.Minecasadi(
        matlab.double(vertices[:, 0].tolist()), 
        matlab.double(vertices[:, 1].tolist()), 
        Position[Id][0], Position[Id][1], Pose[Id],
        T, N, vv, ww)
    x_opt = np.array(x_opt).tolist()
    # x_opt = eng.Minecasadi(
    #     Position[Id][0], Position[Id][1], Pose[Id], 
    #     matlab.double(vertices[:, 0].tolist()), matlab.double(vertices[:, 1].tolist())
    # )
    # x_opt = np.array(x_opt).tolist()

    # 把第一个初始位置给移掉
    x_opt.pop(0)
    # 还原为真实位置
    for positionItem in x_opt:
        # x实际位置
        positionItem[0] = round(positionItem[0] + \
            (vv/ww) * (csd.sin(positionItem[2])), 2)
        # y实际位置
        positionItem[1] = round(positionItem[1] - \
            (vv/ww) * (csd.cos(positionItem[2])), 2)
        # 限制x, y范围
        if   (positionItem[0] <= -xRange): positionItem[0] += 0.01
        elif (positionItem[0] >=  xRange): positionItem[0] -= 0.01
        if   (positionItem[1] <= -yRange): positionItem[1] += 0.01
        elif (positionItem[1] >=  yRange): positionItem[1] -= 0.01
    return x_opt

# 画图画图
def plotTrack(waypoints, Id, centroid, ridges):
    data = np.array(waypoints)
    # 根据ID动态执行
    exec("track_his['{}'][0].append(data[:, 0].tolist())".format(Id))
    exec("track_his['{}'][1].append(data[:, 1].tolist())".format(Id))
    exec("line{}.set_data(track_his['{}'][0], track_his['{}'][1])".format(
        Id, Id, Id))
    exec("ridges_handle{}.set_data(ridges[:, 0], ridges[:, 1])".format(Id))
    if(len(centroid) == len(Position)):
        centroids_handle.set_data(centroid[:, 0], centroid[:, 1])
    
    plt.pause(0.0001)


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
        # print(vertices, centroid)
        centroids.append(centroid)        # 判断点是否在区域内
        path = mpltPath.Path(vertices)
        # inside2 = posList[path.contains_points(posList)][0,:]
        index = np.where(path.contains_points(posList))[0][0]

        outPut = cassingle(vertices, Id[index])
        
        Position[Id[index]] = [pos for pos in outPut[-1][0:2]]
        # print('after', Position[Id[index]])
        # print(Id[index], Position[Id[index]])
        Pose[Id[index]] = round(outPut[-1][-1], 2)

        if(draw):
            plotTrack(outPut, Id[index], np.array(centroids), vertices)

        for timeOrder, item in enumerate(outPut):
            waypoints.append(Waypoint(
                int(Id[index]),
                item[0],
                item[1],
                item[2],
                allcfsTime,
                timeOrder
            ))
    return waypoints


# 中断处理函数，防止不降落
def stop_handler(signum, frame):
    global STOP
    STOP = True
    print("进程被终止，准备降落")

# 位置类


class Waypoint:
    def __init__(self, agent, x, y, rotate, duration, arrival, z=0.5):
        self.agent = agent
        self.x = x
        self.y = y
        self.z = z
        self.rotate = rotate
        self.duration = duration
        self.arrival = arrival

    # 方便根据时间序列进行排序
    def __lt__(self, other):
        return self.arrival < other.arrival

    def __repr__(self):
        return "Ag {}. [{}, {}, {}]".format(self.agent, self.x, self.y, self.z)


if __name__ == "__main__":
    eng = matlab.engine.start_matlab()
    # 设置相应信号处理的handler
    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    # 进行维诺过程
    for counter in range(numIterations):
        # 维诺划分
        vor = voronoi(np.array(
            [Position[key] for key in Position]
        ), box)
        # print(vor.filtered_regions)

        waypoints = matchPoint(vor)
    input("Press Enter to continue...")
