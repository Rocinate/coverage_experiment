import numpy as np

class Shadow:
    def __init__(self, gutter, n):
        self.n = n
        self.gutter = gutter
        self.getBatchDiff()

    # 计算镜像区域位移
    def getBatchDiff(self):
        diff1 = np.random.rand(self.n, 2)
        diff1[:, 0] = diff1[:, 0] / 2
        diff2 = np.random.rand(self.n, 2)
        diff2[:, 0] = diff1[:, 0] / 2
        self.diff1, self.diff2 = diff1, diff2

    # 计算影子信息
    def updateShadow(self, positions):
        n = self.n
        positions[n:2*n, :] = positions[0:n, :] + self.diff1
        positions[n:2*n, 1] = positions[n:2*n, 1] + self.gutter

        positions[2*n:3*n, :] = positions[0:n, :] + self.diff2
        positions[2*n:3*n, 1] = positions[2*n:3*n, 1] - self.gutter

        for flightIndex in range(n, 2*n):
            positions[flightIndex, 0] = positions[flightIndex, 0]
            positions[flightIndex, 1] = self.gutter*2 - positions[flightIndex, 1]

        for flightIndex in range(2*n, 3*n):
            positions[flightIndex, 0] = -16/121 * positions[flightIndex, 0]**2 + \
                8/121 * positions[flightIndex, 0] + \
                1 + positions[flightIndex, 0]
            positions[flightIndex, 1] = positions[flightIndex, 1]

    def getGraphData(self, positions):
        result = np.zeros((self.n*3, 2))
        result[:self.n, :] = positions.copy()
        self.updateShadow(result)
        return result
