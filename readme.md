# crazyswarm实验代码

想要实现运行，可将该文件夹放入swarm的ros_ws/src/crazyswarm/script/部分

## 配置文件

crazyfiles.yaml，和swarm本身运行的配置文件不同，多了一个pose项，这里可以和原本swarm项目的配置文件合并，后续工作可以优化优化。

## 自定义类

| file                                       | part                         |
| ------------------------------------------ | ---------------------------- |
| [borderdVoronoi.py](./borderdVoronoi.py)   | 根据需求魔改后的维诺划分代码 |
| [cassingle.py](./cassingle.py)             | casadi求解器                 |
| [graphController.py](./graphController.py) | 画图                         |
| [mapConvert.py](mapConvert.py)             | 虚拟位置和经纬度映射         |
| [sendJson.py](./sendJson.py)               | 发送Json数据                 |
| [receiveClient.py](./receiveClient.py)     | websocket接收测试端          |
| [CFController.py](./CFController.py)       | 集群控制器                   |

## 实验main函数

| file                                               | part                                                         |
| -------------------------------------------------- | ------------------------------------------------------------ |
| [online_map_sim.py](./online_map_sim.py)           | 演示程序，可以到[网页端](http://45.115.245.21:8081/websocket/#/)观察效果 |
| [online_casadi_pose.py](online_casadi_pose.py)     | 使用casadi进行速度中层控制，单线程                           |
| [online_casadi_thread.py](online_casadi_thread.py) | 使用casadi进行速度中层控制，多线程                           |
| [online_casadi.py](online_casadi.py)               | 使用casadi进行位置高层控制                                   |
| [online_code_debug.py](online_code_debug.py)       | debug用                                                      |
| [online_lloyd.py](online_lloyd.py)                 | 维诺划分，高层位置控制                                       |
| [receiveClient.py](receiveClient.py)               | soceket测试接收端                                            |
| [sendJson.py](sendJson.py)                         | 异步发送数据（只有python3）才能用，ubuntu18.04上面ros对应的版本是python2 |

## TODO

1. 类的使用只有casadi部分，之前的debug和lloyd都没有实现类的封装，心情好可以改善改善
2. 实现中层高层控制的分离，cfcontroller建议添加控制状态代码，目前里面只有中层控制部分
3. 多线程代码中，画图程序暂时失效，如果只是想要看到模拟效果，可以使用python3运行map_sim来查看效果

