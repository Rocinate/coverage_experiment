# -*- coding: UTF-8 -*-
#!/usr/bin/env python
# 卡萨帝求解
from casadi import *
import numpy as np

class Cassingle:
    def __init__(self, lineSpeed, angularSpeed, T, N, xRange, yRange):
        self.lineSpeed = lineSpeed  # 线速度
        self.angularSpeed = angularSpeed  # 角速度
        self.T = T
        self.N = N
        self.xRange = xRange
        self.yRange = yRange
        # 信号场强度
        self.intensity = 1.
        # 插值个数
        self.interpolation = 10
        self.step = 0.05
        # 范围网格
        self.gridData = np.array([
            [round(x, 2), round(y, 2), self.intensity] for x in np.arange(-xRange, xRange, self.step) for y in np.arange(-yRange, yRange, self.step)
        ])

    def update(self, vertices, centroid, Position, Pose):
        # 声明符号变量
        x1 = MX.sym('x1')
        x2 = MX.sym('x2')
        x3 = MX.sym('x3')
        x = vertcat(x1, x2, x3)
        u = MX.sym('u')
        # 动力学模型
        xdot = vertcat(
            (1 - u/self.angularSpeed) * self.lineSpeed * cos(x3), (1 - u/self.angularSpeed) * self.lineSpeed * sin(x3), u)
        # 损失函数
        L = sqrt((x1 - centroid[0]) ** 2 + (x2 - centroid[1]) ** 2)

        # 时间离散化
        M = 4
        DT = self.T/self.N/M
        f = Function('f', [x, u], [xdot, L])
        X0 = MX.sym('X0', 3)
        U = MX.sym('U')
        X = X0
        Q = 0
        for j in range(M):
            k1, k1_q = f(X, U)
            k2, k2_q = f(X + DT/2 * k1, U)
            k3, k3_q = f(X + DT/2 * k2, U)
            k4, k4_q = f(X + DT * k3, U)
            X = X+DT/6*(k1 + 2*k2 + 2*k3 + k4)
            Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q)
        F = Function('F', [X0, U], [X, Q], ['x0', 'p'], ['xf', 'qf'])

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
                upBound[index] = inf
            else:
                lowBound[index] = -inf

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
        VirtualX = round(Position[0] - (self.lineSpeed/self.angularSpeed) * (sin(Pose)), 2)
        VirtualY = round(Position[1] + (self.lineSpeed/self.angularSpeed) * (cos(Pose)), 2)
        VirtualZ = Pose
        Xk = [VirtualX, VirtualY, VirtualZ]

        # 非线性求解器参数
        for k in range(self.N):
            Uk = MX.sym('U_' + str(k))
            w += [Uk]
            lbw += [-0.5]  # 控制u上界
            ubw += [0.5]  # 控制u下界
            w0 += [0]

            Fk = F(x0=Xk, p=Uk)
            Xk = Fk['xf']
            J += Fk['qf']

            # 添加约束
            for i in range(verticesNum):
                g += [(Xk[0] + (self.lineSpeed/self.angularSpeed)*sin(Xk[2]) - verticesX[i]) * (verticesY[i+1] - verticesY[i]) -
                    (Xk[1] - (self.lineSpeed/self.angularSpeed)*cos(Xk[2]) - verticesY[i]) * (verticesX[i+1] - verticesX[i])]

            lbg += lowBound
            ubg += upBound

        # 创建求解器
        prob = {'f': J, 'x': vertcat(*w), 'g': vertcat(*g)}
        # 屏蔽输出，太多啦
        opts = {"ipopt.print_level":0, "print_time": False}
        solver = nlpsol('solver', 'ipopt', prob, opts)
        # solver = nlpsol('solver', 'ipopt', prob)  # 完全输出

        # 求解
        sol = solver(x0=w0, lbx=lbw, ubx=ubw, lbg=lbg, ubg=ubg)
        u_opt = sol['x']

        # 解析求解结果，求解器输出是numpy数组，艹
        x_opt = [[VirtualX, VirtualY, VirtualZ]]
        for k in range(self.N):
            Fk = F(x0=x_opt[-1], p=u_opt[k])
            res = Fk['xf'].full().reshape([3]).tolist()
            x_opt += [res]

        # 把第一个初始位置给移掉
        x_opt.pop(0)

        # 还原为真实位置
        for positionItem in x_opt:
            # x实际位置
            positionItem[0] = round(positionItem[0] + \
                (self.lineSpeed/self.angularSpeed) * (sin(positionItem[2])), 2)
            # y实际位置
            positionItem[1] = round(positionItem[1] - \
                (self.lineSpeed/self.angularSpeed) * (cos(positionItem[2])), 2)
            # 限制x, y范围
            if   (positionItem[0] <= -self.xRange): positionItem[0] += 0.01
            elif (positionItem[0] >=  self.xRange): positionItem[0] -= 0.01
            if   (positionItem[1] <= -self.yRange): positionItem[1] += 0.01
            elif (positionItem[1] >=  self.yRange): positionItem[1] -= 0.01
        return x_opt

    # 计算loss
    def loss(self, position):
        loss = 0.
        for point in self.gridData:
            posList = position - point[0:2]
            lossList = (posList[:, 0] ** 2 + posList[: ,1] ** 2) * point[2]
            loss += round(sqrt(lossList.min()), 2)
        return loss