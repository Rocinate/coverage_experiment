# -*- coding: UTF-8 -*-
#!/usr/bin/env python
class LocateMap:
    def __init__(self, xRange, yRange, lngRange, latRange):
        self.xRange = xRange
        self.yRange = yRange
        self.lngRange = lngRange
        self.latRange = latRange
        self.__getMapProp()

    def __getMapProp(self):
        # 将经纬度范围转换为x, y维度，并计算虚拟坐标和真实坐标之间的缩放参数
        x, y = self.lngRange, self.latRange
        self.zoomX = abs(x[1] - x[0]) / (2 * self.xRange)
        self.zoomY = abs(y[1] - y[0]) / (2 * self.yRange)
        self.offsetX = min(x) - (self.zoomX * -self.xRange)
        self.offsetY = min(y) - (self.zoomY * -self.yRange)

    # 坐标换算
    def xy2lnglat(self, x, y):
        x = x * self.zoomX + self.offsetX
        y = y * self.zoomY + self.offsetY
        return x, y