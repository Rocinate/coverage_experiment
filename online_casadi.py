# -*- coding: UTF-8 -*-
#!/usr/bin/env python
from pycrazyswarm import *
import signal
import numpy as np
import scipy.spatial as sp
import sys
import matplotlib.path as mpltPath
import casadi as csd
import matplotlib.pyplot as plt

eps = sys.float_info.epsilon

## 初始化位置信息
Position = {
    '2': [5, 2.5],
    '4': [6, 2.5],
    '5': [7, 2.5],
    '7': [8, 2.5],
    '8': [9, 2.5],
}

# 无人机姿态数据
Pose = {
  '2': 0.0,
  '4': 0.0,
  '5': 0.0,
  '7': 0.0,
}

# 无人机历史控制量
u_his = {
  '2': 0.0,
  '4': 0.0,
  '5': 0.0,
  '7': 0.0,
}

STOP = False          # 中止标志位
numIterations = 50   # 迭代次数
xRange = 0.5            # 场地长度
yRange = 1              # 场地宽度
box = np.array([-xRange, xRange, -yRange, yRange])  # 场地范围
vv = 0.17  # 速度
ww = 0.1   # 角度
T = 5
N = 5
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
        names['ridges_handle' + i], = plt.plot([], [], 'k-', label='ridges')
    plt.legend()
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
    # 选取在范围内的点
    i = in_box(towers, bounding_box)
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
def cassingle(vertices, centroid, Id):
    Id = int(Id)
    # 声明符号变量
    x1 = csd.SX.sym('x1')
    x2 = csd.SX.sym('x2')
    x3 = csd.SX.sym('x3')
    x = csd.vertcat(x1, x2, x3)
    u = csd.SX.sym('u')
    # 动力学模型
    xdot = csd.vertcat(
        (1 - u/ww) * vv * csd.cos(x3), (1 - u/ww) * vv * csd.sin(x3), u)
    # 损失函数
    L = csd.sqrt((x1 - centroid[0]) ** 2 + (x2 - centroid[1]) ** 2)

    # 时间离散化
    M = 4
    DT = T/N/M
    f = csd.Function('f', [x, u], [xdot, L])
    X0 = csd.MX.sym("X0", 3)
    U = csd.MX.sym('U')
    X = X0
    Q = 0
    for j in range(M):
        k1, k1_q = f(X, U)
        k2, k2_q = f(X + DT/2 * k1, U)
        k3, k3_q = f(X + DT/2 * k2, U)
        k4, k4_q = f(X + DT * k3, U)
        X = X+DT/6*(k1 + 2*k2 + 2*k3 + k4)
        Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)
    F = csd.Function('F', [X0, U], [X, Q], ['x0', 'p'], ['xf', 'qf'])

    # 为了计算点和线的位置情况，在顶点数组末尾拼接上第二个值
    verticesX = np.append(vertices[:, 0], vertices[1, 0])
    verticesY = np.append(vertices[:, 1], vertices[1, 1])

    # 边界判断
    verticesNum = vertices.shape[0] - 1
    # 高低边界
    lowBound = [0] * verticesNum
    upBound = [0] * verticesNum

    # 判断点与边界线的关系
    for index in range(verticesNum):
        if(((verticesX[index + 2] - verticesX[index]) * (verticesY[index + 1] - verticesY[index]) -
            (verticesY[index + 2] - verticesY[index]) * (verticesX[index + 1] - verticesX[index])) > 0):
            upBound[index] = csd.inf
        else:
            lowBound[index] = -csd.inf

    # 初始化非线性求解器参数
    w = []
    w0 = []
    lbw = []
    ubw = []
    J = 0
    g = []
    lbg = []
    ubg = []

    # 转换为虚拟姿态
    VirtualX = Position[str(Id)][0] - (vv/ww) * (csd.sin(Pose[str(Id)]))
    VirtualY = Position[str(Id)][1] + (vv/ww) * (csd.cos(Pose[str(Id)]))
    VirtualZ = Pose[str(Id)]
    Xk = [VirtualX, VirtualY, VirtualZ]

    # 非线性求解器参数
    for k in range(N):
        Uk = csd.MX.sym('U_' + str(k))
        w += [Uk]
        lbw += [-0.5]  # 控制u上界
        ubw += [0.5]  # 控制u下界
        w0 += [0]
    #   w0 += [u_his[str(Id)]]

        Fk = F(x0=Xk, p=Uk)
        Xk = Fk['xf']
        J += Fk['qf']

        # 添加约束
        for i in range(verticesNum):
            g += [(Xk[0] + (vv/ww)*csd.sin(Xk[2]) - verticesX[i]) * (verticesY[i+1] - verticesY[i]) -
                  (Xk[1] - (vv/ww)*csd.cos(Xk[2]) - verticesY[i]) * (verticesX[i+1] - verticesX[i])]

        lbg += lowBound
        ubg += upBound

    # 创建求解器
    prob = {'f': J, 'x': csd.vertcat(*w), 'g': csd.vertcat(*g)}
    # 屏蔽输出，太多啦
    opts = {"ipopt.print_level":0, "print_time": False}
    solver = csd.nlpsol('solver', 'ipopt', prob, opts)

    # 求解
    sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
    u_opt = sol['x']

    # 解析求解结果，求解器输出是numpy数组，艹
    x_opt = [[VirtualX, VirtualY, VirtualZ]]
    for k in range(N):
        Fk = F(x0=x_opt[-1], p=u_opt[k])
        res = Fk['xf'].full().reshape([3]).tolist()
        x_opt += [res]

    # 历史控制量
    u_his[str(Id)] = u_opt[-1]

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
        centroids.append(centroid)
        # 判断点是否在区域内
        path = mpltPath.Path(vertices)
        # 通过位置判断匹配质心和无人机序号
        index = np.where(path.contains_points(posList))[0][0]
        outPut = cassingle(vertices, centroid, Id[index])

        # 减去一个小值，避免出界抽风
        Position[Id[index]] = [pos for pos in outPut[-1][0:2]]
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
    def __init__(self, agent, x, y, rotate, duration,arrival, z=0.5):
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
    # for counter in range(0, numIterations):
    for counter in range(numIterations):
        # 维诺划分
        vor = voronoi(np.array(
            [Position[key] for key in Position]
        ), box)

        # 根据计算出来的维诺质心进行无人机匹配
        waypoints = matchPoint(vor)
        # print(waypoints)

        # 根据到达排序进行排序
        waypoints.sort()

        # 遍历无人机，发送指令
        for waypoint in waypoints:
            pos = [waypoint.x, waypoint.y, waypoint.z]
            # print(waypoint.agent, pos)
            cf = allcfs.crazyfliesById[waypoint.agent]
            cf.goTo(pos, waypoint.rotate, waypoint.duration)

        # 通过ros定时器去休眠
        timeHelper.sleep(waypoints[0].duration)

        while True:
            arrived = 0
            for waypoint in waypoints:
                cf = allcfs.crazyfliesById[waypoint.agent]
                m_position = cf.position()
                # 计算与目标点的距离，判断移动是否执行完毕
                error = np.linalg.norm(
                    np.array(Position[waypoint.agent]) - m_position[0:2])
                if(error < 0.1):
                    arrived += 1
                    # 更新下一轮迭代所需要的位置信息
                    Position[waypoint.agent] = m_position[0:2].tolist()

            # 全部无人机到位，退出循环
            if(arrived == len(Position.keys())):
                print("iteration {} pass, computing next position.".format(counter))
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
    # 避免画图终止
    input("Press Enter to continue...")