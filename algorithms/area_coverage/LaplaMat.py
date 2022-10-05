# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import numpy as np
import math
from scipy.spatial.distance import pdist,squareform

#定义计算拉普拉斯矩阵的函数
#输入：x为N*n的状态矩阵，N为智能体数量，n为状态维度；R为通信范围
#输出：n*n的拉普拉斯矩阵L，邻接矩阵A，距离矩阵d
def L_Mat(X,R,deta):
   value=-(R**2/math.log(deta))
   d = squareform(pdist(X,'euclidean'))    
   N = len(X)
   A = np.zeros((N,N))
   for i in range(N):
       for j in range(N):
           if(i==j):
               continue
           if(d[i,j]<=R):
               A[i,j] = math.exp(-d[i,j]**2/(value))
           else:
               A[i,j] = 0
           A[i,j] = A[i,j] 
   A = A-np.diag(np.diag(A))
   D = np.diag(np.sum(A,axis=1))
   L = D-A
   
   return L,A,d
   


