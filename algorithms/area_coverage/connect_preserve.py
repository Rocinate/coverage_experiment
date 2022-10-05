# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import numpy as np
import math

#定义计算连通保持控制律函数
def con_pre(lamde2,x2,position,d,A,R,deta,Epsilon):
    '''
    #此控制律保持连通度不低于设定阈值。
    #输入：lamde2表示nx1特征值估计向量，x2表示对应的特征向量估计nx1。
    #x表示位置矩阵
    #d表示距离平方矩阵nxn，A表示邻接矩阵，R表示通信半径，deta表示通信半径下的权值
    #Epsilon表示连通度最小阈值。
    '''
    value = -(R**2/math.log(deta))/2
    x = np.array([position[key] for key in position]) 
    Idlist = list(position.keys())
    uc = {}
    n = len(lamde2)    # 将ue0数组与无人机的编号匹配，输出字典uc
    uc0 = np.zeros((n,2))
    for i in range(n):
        if lamde2[i]<Epsilon:
            a1=-50000
        else:
            a1 = -1/(math.sinh(lamde2[i]-Epsilon)**2)
        a2x = 0
        a2y = 0
        idx = [idx for (idx, val) in enumerate(d[i]) if val <= R]
        idx.remove(i)
        for j in range(len(idx)):
            agent = idx[j]
            a2x = a2x+(-(A[i,agent])/value)*((x[i,0])-(x[agent,0]))*((x2[i]-x2[agent])**2)
            a2y = a2y+(-(A[i,agent])/value)*((x[i,1])-(x[agent,1]))*((x2[i]-x2[agent])**2)
        ucx = -a1*a2x
        ucy = -a1*a2y
        uc0[i,:] = np.vstack((ucx,ucy)).T
   # 将ue0数组与无人机的编号匹配，输出字典uc
    Idlist = list(position.keys())
    uc = {K:V for K in Idlist
              for V in range(1)}
    t = 0
    for j in Idlist:
        uc[j] = uc0[t,:]
        t = t+1
   
    return uc

       

   

