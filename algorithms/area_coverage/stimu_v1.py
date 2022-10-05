# -*- coding: UTF-8 -*-
# !/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'SimHei'
plt.rcParams['axes.unicode_minus'] =False
# 自定义模块
from LaplaMat import L_Mat
from connect_preserve import con_pre
from coverage_control import con_control
from wayplot import way_plot, radar_plot
# 初始位置信息
Position = {
    '1': [-0.2, 3],
    '2': [0.2, 3],
    '3': [-0.2, 2.7],
    '4': [0.2, 2.7],
    '5': [0.6, 2.4],
    '6': [0.3, 2.4],
    '7': [-0.3, 2.4],
    '8': [-0.6, 2.4],
}

xRange_min = -3.2  # 场地长度
xRange_max = 3.8
yRange_min = -3.2
yRange_max = 3.8
box = np.array([xRange_min, xRange_max, yRange_min, yRange_max])  # 场地范围

# 此任务中的常量参数
R = 2  # 通信半径
deta = 0.1  # 通信边界处的边权大小
Epsilon = 0.1  # 最小代数连通度
vmax = 0.1  # 最大速度（用于限幅）
dt = 1  # 控制器更新频率
totalTime = 80
T = 5
numIterations = int((totalTime / dt)) + 1  # 迭代次数
warnEpsilon = 0.6
draw = 1  # 是否绘图,为1时绘图
radar = np.loadtxt(open('zhongchuang_0.5.csv'), delimiter=',', skiprows=0, dtype=np.float64)  # 导入信号场数据
# radar = np.loadtxt('data-0.3-2.txt')
lamde_h=[]
if __name__ == "__main__":
    # 初始绘图设置
    if (draw == 1):
        radar_plot(radar, box)
        plt.show()

    Idlist = list(Position.keys())
    n = len(Position)
    t = 0

    # 覆盖控制
    for counter in range(numIterations):
        # 计算连通度
        posList = np.array([Position[key] for key in Position])
        L, A, d = L_Mat(posList, R, deta)
        value_h, vector_h = np.linalg.eig(L)
        value = value_h[np.argsort(value_h)]
        vector = vector_h[:, np.argsort(value_h)]
        t = round(t, 1)
        print(str(t) + 's connectivity is::' + str(value[1]))
        # 计算 ue
        ue = con_control(Position, box, radar)
        # 限幅处理
        for i in Idlist:
            ue_norm = np.linalg.norm(ue[i])
            if ue_norm > vmax:
                ue[i] = vmax * ue[i] / ue_norm

        # 计算uc
        lamde2est = np.ones((n, 1)) * value[1]
        v2 = vector[:, 1]
        uc = con_pre(lamde2est, v2, Position, d, A, R, deta, Epsilon)
        # 限幅处理
        for j in Idlist:
            uc_norm = np.linalg.norm(uc[j])
            if uc_norm > 1.2*vmax:
                uc[j] = 1.2*vmax * uc[j] / uc_norm

        # 总控制律
        u = {}
        # for i in Idlist:
        #     u[i] = ue[i]
        if value[1] >= warnEpsilon:
            for i in Idlist:
                u[i] = ue[i]
        else:
            if t % T == 0 and t <= 70:
                ue1 = []
                for key in ue.keys():
                    ue1.append(ue[key])
                ue1 = np.reshape(ue1, (n, 2))
                ue2 = np.sum(ue1, axis=0)
                # 限幅
                ue2_norm = np.linalg.norm(ue2)
                if ue2_norm > 3*vmax:
                    ue2 = 3*vmax * ue2 / ue2_norm
                for i in Idlist:
                    u[i] = ue2
            else:
                for i in Idlist:
                    u[i] = ue[i] + uc[i]
        # 绘制当前时刻无人机的位置
        if draw == 1:
            way_plot(posList[:, 0], posList[:, 1], radar, box)

        # 位置更新
        uList = np.array([u[key] for key in u])
        New_posList = posList + uList * dt
        pt = 0
        for id2 in Idlist:
            Position[id2] = New_posList[pt].tolist()
            pt = pt + 1
        # 时间更新
        t = t + dt
        lamde_h.append(value[1])

    plt.show()
    plt.figure(figsize=(10, 5))
    plt.plot(range(1, numIterations+1, 1), lamde_h)
    plt.xlabel("迭代次数")
    plt.ylabel("代数连通度")
    plt.ylim([0,3])
    plt.show()
