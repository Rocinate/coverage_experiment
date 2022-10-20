import numpy as np

coverAngles = np.array([3.19604496, 3.28541413])
angleStart, angleEnd = np.pi*165/180, np.pi*195/180
cov = 5.0/180*np.pi

overlapAngles = 0.0

# 计算重合角度
for idx in range(len(coverAngles)):
    if idx == 0:
        if coverAngles[idx] - angleStart < cov / 2:
            overlapAngles += (angleStart - coverAngles[idx] + cov / 2)
        continue

    if coverAngles[idx] - coverAngles[idx-1] < cov:
        overlapAngles += (coverAngles[idx - 1] + cov - coverAngles[idx])

    if idx == len(coverAngles) -1 and angleEnd - coverAngles[idx] < cov / 2:
        overlapAngles += (coverAngles[idx] + cov / 2 - angleEnd)

print(overlapAngles)