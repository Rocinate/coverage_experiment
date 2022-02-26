# -*- coding: UTF-8 -*-
#!/usr/bin/env python
import yaml
import json
import numpy as np
from mapConvert import LocateMap
from borderdVoronoi import Vor
from cassingle import Cassingle
from graphController import Graph
import time
# from sendJson import SendJson

# 读取无人机位置配置
with open("crazyfiles.yaml", "r") as f:
    data = yaml.load(f, Loader=yaml.FullLoader)

allCrazyFlies = data['files']


STOP = False
numIterations = 60
xRange = 0.75
yRange = 1.
box = np.array([-xRange, xRange, -yRange, yRange])  # 场地范围
lineSpeed = 0.05
angularSpeed = 0.5
T = 3.0
N = 6
draw = True # 是否画图
txt = True  # 是否打印json
allcfsTime = T/N


# 取石人公园和中医大省医院为范围
latRange = (30.681858,  30.672805)
lngRange = (104.036209, 104.047042)

def print2txt(waypoints):
    # 将输出打印到txt中，方便调试
        with open("origin.json", "w") as f:    #打开文件
            f.write(waypoints)
            f.write(',\n')

if __name__ == "__main__":
    # 时间统计
    start = time.perf_counter()

    locateMap = LocateMap(xRange, yRange, lngRange, latRange)

    vor = Vor(box, lineSpeed, angularSpeed)

    cassingle = Cassingle(lineSpeed, angularSpeed, T, N, xRange, yRange, 'Euclidean', 4)
    
    graph = Graph([str(cf['Id']) for cf in allCrazyFlies], xRange, yRange)

    allWaypoints = []

    for counter in range(numIterations):
        print("epoch: {}, progress: {}%".format(
            counter,
            round(float(counter)/numIterations * 100, 2)
        ))
        # 更新维诺划分
        vorResult = vor.updateVor(allCrazyFlies)

        waypoints = []

        for flie in vorResult:
            # 找出对应Id储存在allcrazyfiles中的索引
            [matchIndex] =  [index for (index, item) in enumerate(allCrazyFlies) if item['Id'] == flie['Id']]
            # casadi运算下一步位置
            outPut = cassingle.update(
                flie['vertices'],
                flie['centroid'],
                allCrazyFlies[matchIndex]['Position'],
                allCrazyFlies[matchIndex]['Pose']
            )

            allCrazyFlies[matchIndex]['Position'] = [pos for pos in outPut[-1][0:2]]
            allCrazyFlies[matchIndex]['Pose'] = round(outPut[-1][-1], 4)
            
            draw and graph.updateTrack(
                np.array(outPut), 
                allCrazyFlies[matchIndex]['Id']
            )

            # 转换轨迹为经纬度
            waypoints.append({
                'id': allCrazyFlies[matchIndex]['Id'],
                'points': []
            })

            for item in outPut:
                lng, lat = locateMap.xy2lnglat(item[0], item[1])
                waypoints[len(waypoints) - 1]['points'].append({
                    'position': item,
                    'lng': lng,
                    'lat': lat
                })

        # 更新维诺质心
        draw and graph.updateCentroid(
            np.array([cf['centroid'] for cf in vorResult])
        )
        
        # 使用虚拟位置更新维诺边界
        virtualResult = vor.virtualVor(allCrazyFlies)
        draw and graph.updateRidges(virtualResult)

        # 将虚拟边界转换为经纬度
        for flie in waypoints:
            [matchIndex] = [
                index for (index, item) in enumerate(virtualResult) if item['Id'] == flie['id']
            ]
            flie['voronoiDiagram'] = []
            flie['centroid'] = []
            for vertice in virtualResult[matchIndex]['vertices']:
                lng, lat = locateMap.xy2lnglat(vertice[0], vertice[1])
                flie['voronoiDiagram'].append({
                    'lng': lng,
                    'lat': lat
                })
            centroid = virtualResult[matchIndex]['centroid']
            lng, lat = locateMap.xy2lnglat(centroid[0], centroid[1])
            flie['centroid'].append({
                'lng': lng,
                'lat': lat
            })

        # 计算loss
        virtualPosition = vor.virtualPosition(allCrazyFlies)
        lossList = []
        for i in range(len(waypoints[0]['points'])):
            lossList.append(
                cassingle.loss(np.array(
                    [cf['points'][i]['position'][0:2] for cf in waypoints]
                ))  
            )
        waypoints.append({
            "lossFunction": lossList
        })
        
        # 将输出打印到txt中，方便调试
        txt and print2txt(json.dumps(waypoints))

        allWaypoints.append(waypoints)

    # for waypoints in allWaypoints:
    #     SendJson(json.dumps(waypoints))

    end = time.perf_counter()
    print("time consumed: {}s".format(end-start))
    # 防止图像关闭
    input("Press Enter to continue...")