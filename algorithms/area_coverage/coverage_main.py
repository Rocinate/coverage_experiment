# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import sys
import os
currentUrl = os.path.dirname(__file__)
parentUrl = os.path.abspath(os.path.join(currentUrl, os.pardir))
sys.path.append(parentUrl)
parentUrl1 = os.path.abspath(os.path.join(parentUrl, os.pardir))

from pycrazyswarm import *
import signal
import numpy as np
import thread
import csv
import time
import matplotlib.pyplot as plt
plt.rcParams['font.family'] = 'SimHei'
plt.rcParams['axes.unicode_minus'] =False
#自定义模块
from LaplaMat import L_Mat
from connect_preserve import con_pre
from coverage_control import con_control
from wayplot import way_plot,radar_plot
#初始位置信息
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
STOP = False          # 中止标志位
speed =  0.05
xRange_min = -3.2  # 场地长度
xRange_max = 3.8
yRange_min = -3.2
yRange_max = 3.8
box = np.array([xRange_min, xRange_max, yRange_min, yRange_max])  # 场地范围

#此任务中的常量参数
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
# 以固定速度计算所有无人机飞到标准点所需要的时间
def getTime(positions, centroids):
    maxTime = 0.0
    for index, item in enumerate(positions):
        distance = np.linalg.norm(np.array(item) - np.array(centroids[index]))
        time = distance / speed
        maxTime = maxTime if(maxTime > time) else time
    maxTime2=maxTime if(maxTime>1) else 1
    return maxTime2

# 匹配维诺区域与无人机，返回无人机位置序列
def matchPoints(positions,u):
    waypoints = []
    posList_h = np.array([positions[key] for key in positions])
    Id = list(positions.keys())
    uList = np.array([u[key] for key in u])
    posList_new = posList_h + uList*dt
    pt = 0
    for i in Id:
        positions[i] = posList_new[pt].tolist()
        pt=pt+1

    allcfsTime = getTime(posList_h.tolist(), posList_new)

    for key in positions.keys():
        waypoints.append(Waypoint(
                int(key),
                positions[key][0],
                positions[key][1],
                allcfsTime
            ))
    return positions,waypoints

# 中断处理函数，防止不降落
def stop_handler(signum, frame):
    global STOP
    STOP = True
    print("进程被终止，准备降落")

#save data
def wt_csv(swarm,gap):
        status=1
        with open("allcfsPoistion.csv",'a') as f:
            csvName=["Time","id","x[m]","y[m]","z[m]"]
            csv_write=csv.writer(f)
            csv_write.writerow(csvName)
            allcfs_new = swarm.allcfs
            timeHelper=swarm.timeHelper
            while True:
                if status:
                    timeNow=timeHelper.time()
                    for id in allcfs_new.crazyfliesById:
                        try:
                            pos = allcfs_new.crazyfliesById[id].position()[0:3]
                        except: 
                            print("cf"+str(id)+" stops getting message!")
                            status=0
                            continue
                        data = [timeNow,id]+pos.tolist()
                        csv_write.writerow(data)
                        #np.savetxt(f,np.column_stack(data),delimiter=',')
                    time.sleep(gap)
                else:
                    f.close()
                    break

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
    swarm = Crazyswarm(crazyflies_yaml=parentUrl1+"/launch/crazyflies.yaml")
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs
    thread.start_new_thread(wt_csv,(swarm,0.1))
    # 所有无人机同时起飞
    allcfs.takeoff(targetHeight=0.5, duration=2.0)
    # 等待2秒
    timeHelper.sleep(2.0)
    
    #初始绘图设置
    if (draw == 1):
        radar_plot(radar,box)
    
    Idlist = list(Position.keys())
    n = len(Position)
    t = 0
    lamde2_h=[]
    time_h=[]
    # 覆盖控制
    for counter in range(numIterations):
    #计算连通度
        posList = np.array([Position[key] for key in Position])
        L,A,d = L_Mat(posList,R,deta)
        value_h, vector_h = np.linalg.eig(L)
        value = value_h[np.argsort(value_h)]  
        vector = vector_h[:,np.argsort(value_h)]
        t = round(t,1) 
        print(str(t)+"s connectivity is:"+str(value[1])) 
        lamde2_h.append(value[1])
        time_h.append(t)   

    # 计算 ue
        ue = con_control(Position,box,radar)
        #限幅处理
        for i in Idlist:
             ue_norm = np.linalg.norm(ue[i])
             if (ue_norm > vmax):
                 ue[i] = vmax*ue[i]/ue_norm

    # 计算uc
        lamde2est = np.ones((n,1))*value[1]
        v2 = vector[:,1]
        uc = con_pre(lamde2est,v2,Position,d,A,R,deta,Epsilon)
        #限幅处理
        for j in Idlist:
            uc_norm=np.linalg.norm(uc[j])
            if (uc_norm >1.2* vmax):
                uc[j] = 1.2*vmax*uc[j]/uc_norm

    #总控制律
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
                if ue2_norm > 3 * vmax:
                    ue2 = 3 * vmax * ue2 / ue2_norm
                for i in Idlist:
                    u[i] = ue2
            else:
                for i in Idlist:
                    u[i] = ue[i] + uc[i]
                
        
    # 绘制当前时刻无人机的位置
        if (draw == 1):
             way_plot(posList[:,0] ,posList[:,1],radar,box)
        
    # 位置更新
        Position,waypoints = matchPoints(Position,u)
    # 时间更新
        t = t+dt

    # 遍历无人机，发送指令
        for waypoint in waypoints:
            pos = [waypoint.x, waypoint.y, waypoint.z]
            #print(waypoint.agent, pos,waypoint.duration)
            cf = allcfs.crazyfliesById[waypoint.agent]
            cf.goTo(pos, 0, waypoint.duration)
        
    # 通过ros定时器去休眠
        timeHelper.sleep(waypoints[0].duration)
        
        while True:
            arrived = 0
            for waypoint in waypoints:
                cf = allcfs.crazyfliesById[waypoint.agent]
                error = np.linalg.norm(np.array(Position[str(waypoint.agent)]) - cf.position()[0:2])
                if(error < 0.1): 
                    arrived += 1
                    # 更新下一轮迭代所需要的位置信息
                    Position[str(waypoint.agent)] = cf.position()[0:2]

            # 全部无人机到位，退出循环
            if(arrived == len(Position.keys())):
                print("iteration {} pass.".format(counter))
                print("time {} pass.".format(t))
                break

    # 收到终止信号，停止运行
        if(STOP):
            # 原地降落
            allcfs.land(targetHeight=0.02, duration=2.0)
            timeHelper.sleep(2.0)
            thread.exit()
            break

    # 执行完毕，降落
    allcfs.land(targetHeight=0.02, duration=2.0)
    timeHelper.sleep(2.0)
    with open("allcfslamde2.csv",'a') as f:
            csvName=["Time","lamde2"]
            csv_write=csv.writer(f)
            csv_write.writerow(csvName)
            for i in range(len(time_h)):
                data=[time_h[i],lamde2_h[i]]
                csv_write.writerow(data)

    plt.figure(figsize=(10, 5))
    plt.plot(range(len(time_h)), lamde2_h)
    plt.xlabel("迭代次数")
    plt.ylabel("代数连通度")
    plt.ylim([0, 3])
    plt.show()
