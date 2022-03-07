## 此文件夹为实验代码

### 自定义类

| file                                       | part                         |
| ------------------------------------------ | ---------------------------- |
| [borderdVoronoi.py](./borderdVoronoi.py)   | 根据需求魔改后的维诺划分代码 |
| [cassingle.py](./cassingle.py)             | casadi求解器                 |
| [graphController.py](./graphController.py) | 画图                         |
| [mapConvert.py](mapConvert.py)             | 虚拟位置和经纬度映射         |
| [sendJson.py](./sendJson.py)               | 发送Json数据                 |
| [receiveClient.py](./receiveClient.py)     | websocket接收测试端          |

### 实验main函数

| file                                                 | part                                                         |
| ---------------------------------------------------- | ------------------------------------------------------------ |
| [online_map_sim.py](./online_map_sim.py)             | 演示程序，需要[web界面](http://45.115.245.21:8081/websocket/#/)配合 |
| [online_casadi_pose.py](./online_casadi_pose.py)     | 多进程覆盖控制程序，--local本地模拟，--record记录路径到record.txt，--load从record.txt读取路径 |
| [online_casadi_thread.py](./online_casadi_thread.py) | 多线程覆盖控制程序                                           |
|                                                      |                                                              |
