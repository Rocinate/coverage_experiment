import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from algorithms.LaplaMat import L_Mat
from algorithms.connect_preserve import con_pre
from algorithms.ccangle import ccangle

plt.rcParams["font.sans-serif"] = ["SimHei"]  # 设置字体
plt.rcParams["axes.unicode_minus"] = False  # 正常显示负号

# 覆盖扇面作图
r = 3000  # 雷达半径  #速度167m/s
circleX, circleY = 21000, 5500  # 雷达中心
angleStart, angleEnd = np.pi*165/180, np.pi*195/180  # 扇面覆盖范围30°
cov = 2/180*np.pi  # 单机覆盖角度

intNum = 20  # 覆盖扇面插值数
angleList = np.linspace(angleStart, angleEnd, intNum)  # 计算覆盖扇面位置,用于作图

# 扇形点位，添加起点保证图像闭合
xList = [circleX] + [circleX + r *
                     np.cos(angle) for angle in angleList] + [circleX]
yList = [circleY] + [circleY + r *
                     np.sin(angle) for angle in angleList] + [circleY]

_, ax = plt.subplots()
# 动态绘图
plt.ion()
plt.title("无人机轨迹")
plt.plot(xList, yList)
plt.xlim((0, 21000))
plt.ylim((0, 11000))

n = 20  # 无人机数量/batch
batch = 1  # 批次
np.random.seed(6)  # 随机种子设定，保证可复现

positions = np.random.rand(n * batch, 2) * 900
positions[:, 1] += 5000

plt.plot([1000, 1000], [0, 11000], linestyle='--',
         color='orange', linewidth=0.5)
plt.plot([18000, 18000], [0, 11000], linestyle='--',
         color='orange', linewidth=0.5)

# 无人机初始角度
Angle = np.pi + \
    np.arctan((circleY - positions[:, 1]) / (circleX - positions[:, 0]))

agentHandle = plt.scatter(
    positions[:, 0], positions[:, 1], marker=">", edgecolors="blue", c="white")

# 覆盖扇面作图
verHandle = [None] * n * batch
for index in range(n * batch):
    # 初始化
    patch = patches.Polygon([
        [circleX + r * np.cos(Angle[index]-cov/2), circleY + r * np.sin(Angle[index]-cov/2)],
        [circleX + r * np.cos(Angle[index]+cov/2), circleY + r * np.sin(Angle[index]+cov/2)],
        [circleX, circleY]
    ], fill=False)
    verHandle[index] = ax.add_patch(patch)

# 参数设置
R = 500  # 通信半径
delta = 0.1  # 通信边界边权大小，越小效果越好
epsilon = 0.1  # 最小代数连通度
vMax = 27.8  # 连通保持最大速度（用于限幅）
veAngle = np.zeros(n) # 无人机朝向角
totalTime = 1200  # 仿真总时长
dt = 1.  # 控制器更新频率
epochNum = int(np.floor(totalTime / dt))

# 无人机位置，角度数据保存
Px_h = np.zeros((n*batch, epochNum))
Py_h = np.zeros((n*batch, epochNum))
Angle_h = np.zeros((n*batch, epochNum))
Px_h[:, 0] = positions[:, 0]
Py_h[:, 0] = positions[:, 1]
Angle_h[:, 0] = Angle

# 日志向量定义（记录控制量）
ue_hx = np.zeros((n*batch, epochNum))
ue_hy = np.zeros((n*batch, epochNum))
ue_hx[:, 0] = vMax
uc_hx = np.zeros((n*batch, epochNum))
uc_hy = np.zeros((n*batch, epochNum))
u_hx = np.zeros((n*batch, epochNum))
u_hy = np.zeros((n*batch, epochNum))
u_hx[:, 0] = ue_hx[:, 0] + uc_hx[:, 0]
u_hy[:, 0] = ue_hy[:, 0] + uc_hy[:, 0]
veAngle_h = np.zeros((n*batch, epochNum))
lambda_h = np.zeros(epochNum)

activate = np.ones(n) # 判断无人机是否参与覆盖，参与赋值1，不参与复制0
time = 0
timeCov = np.zeros(epochNum) # 储存t时刻覆盖率超过85%的概率
coverage = np.zeros(epochNum)

# 初始损失和连通度计算
L, A, d = L_Mat(positions, R, delta)

value, vectors = np.linalg.eig(L)
# 从小到大对特征值进行排序
index = np.argsort(value)
vectors = vectors[:, index]
value = value[index]

lambda_h[0] = value[1]
print("1时刻的连通度为{}".format(value[1]))
plt.show()

for epoch in range(epochNum):
    # print(value)
    if positions[:, 0].max() > 18000:
        break
    else:
        activate = np.ones(n)
        # 无人机位置
        # edges = np.array([[-150, 320, -150, -150], [-100, 0, 150, -100]])
        # n_edges = np.zeros((2*n, 4))
        # for k in range(n):
        #     Rt = [[np.cos(veAngle_h[k, epoch]), -np.sin(veAngle_h[k, epoch])],
        #           [np.sin(veAngle_h[k, epoch]), np.cos(veAngle_h[k, epoch])]]
        #     for i in range(4):
        #         n_edges[2*k:2*(k+1), i] = np.dot(Rt, edges[:,i]) + positions[k, :]
        agentHandle.set_offsets(positions)
        plt.pause(0.00001)
        # ti.sleep(0.5)
        # 角度覆盖控制率
        ue = ccangle(
            positions,
            Angle_h[:, epoch], ue_hy[:, epoch], veAngle_h[:, epoch],
            angleStart, angleEnd, R, vMax, cov)
        # print(ue)
        # break
        ue_hx[:, epoch + 1] = ue[:, 0]
        ue_hy[:, epoch + 1] = ue[:, 1]

        # 判断无人机控制率是否改变，使无人机轨迹平滑
        # print(np.abs(ue_hx[:, epoch+1] - ue_hx[:,epoch]))
        changeIndex = np.abs(ue_hx[:, epoch+1] - ue_hx[:,epoch]) < 0.01
        ue_hx[changeIndex, epoch+1] = ue_hx[changeIndex, epoch]
        changeIndex = np.abs(ue_hy[:, epoch+1] - ue_hy[:,epoch]) < 0.01
        ue_hy[changeIndex, epoch+1] = ue_hy[changeIndex, epoch]

        # 分段连通约束控制
        features  = np.ones(n) * value[1]
        featureVec = vectors[:, 1]

        uc = con_pre(features, featureVec, positions, d, A, R, delta, epsilon)
        # 限幅
        for agent in range(n):
            dist = np.linalg.norm(uc[agent, :])
            if dist > vMax:
                uc[agent, :] = vMax * uc[agent, :] / dist
        uc_hx[:, epoch+1] = uc[:, 0]
        uc_hy[:, epoch+1] = uc[:, 1]

        # 总控制
        # u = 3 * uc + ue
        u = 3 * uc + ue

        # for agent in range(n):
        #     dist = np.linalg.norm(u[agent, :])
        #     if dist > vMax:
        #         u[agent, :] = vMax * u[agent, :] / dist
        for agent in range(n):
            dist = np.linalg.norm(u[agent, :])
            if dist > vMax:
                u[agent, :] = vMax * u[agent, :] / dist

        # 控制率叠加
        u_hx[:, epoch + 1] = u[:, 0]
        u_hy[:, epoch + 1] = u[:, 1]
        Px_h[:, epoch + 1] = Px_h[:, epoch] + u[:, 0] * dt
        Py_h[:, epoch + 1] = Py_h[:, epoch] + u[:, 1] * dt
        Angle_h[:, epoch + 1] = np.pi + np.arctan((circleY - Py_h[:, epoch+1]) / (circleX - Px_h[:, epoch + 1]))
        Angle = Angle_h[:, epoch + 1]
        veAngle_h[:, epoch + 1] = np.arcsin(u_hy[:, epoch + 1] / vMax)

        # 判断无人机是否执行覆盖任务
        changeIndex = Px_h[:, epoch] <= 1000
        activate[changeIndex] = 0
        u_hx[changeIndex, epoch+1] = u_hx[changeIndex, epoch]
        u_hy[changeIndex, epoch+1] = u_hy[changeIndex, epoch]
        Px_h[changeIndex, epoch+1] = Px_h[changeIndex, epoch] + u_hx[changeIndex, epoch+1]*dt
        Py_h[changeIndex, epoch+1] = Py_h[changeIndex, epoch] + u_hy[changeIndex, epoch+1]*dt
        Angle_h[changeIndex, epoch+1] = np.pi + np.arctan((circleY - Py_h[changeIndex, epoch+1]) / (circleX - Px_h[changeIndex, epoch + 1]))
        Angle[changeIndex] = Angle_h[changeIndex, epoch+1]
        veAngle_h[changeIndex, epoch+1] = np.arcsin(u_hy[changeIndex, epoch + 1] / vMax)


        # 更新位置
        positions[:, 0] = Px_h[:, epoch + 1]
        positions[:, 1] = Py_h[:, epoch + 1]
        # 计算下一时刻的连通度
        L, A, d = L_Mat(positions, R, delta)
        value, vectors = np.linalg.eig(L)
        # 从小到大对特征值进行排序
        index = np.argsort(value)
        vectors = vectors[:, index]
        value = value[index]

        print("{}时刻的连通度为{}".format(epoch + 1, value[1]))
        lambda_h[epoch+1] = value[1]

        # 覆盖率计算
        overlapping_angle = 0 # 覆盖重叠角度
        # 找到参与覆盖并且在覆盖角度中的agent，只选取在覆盖范围角度内的
        angleSorted = sorted([Angle[index] for index, value in enumerate(activate) if value and Angle[index] > angleStart and Angle[index] < angleEnd])
        if len(angleSorted) == 0:
            coverage[epoch] = 0
        else:
            if angleSorted[0] - angleStart < cov / 2:
                overlapping_angle += angleStart - angleSorted[0] + cov / 2

            for index, angle in enumerate(angleSorted):
                # 跳过首个处理过的角度
                if index == 0:
                    continue

                if angle - angleSorted[index - 1] < cov:
                    overlapping_angle += angleSorted[index - 1] + cov - angle

            # 处理尾部
            if angleEnd - angleSorted[-1] < cov / 2:
                overlapping_angle += angleSorted[-1] + cov / 2 - angleEnd

            coverage[epoch] = (cov * len(angleSorted) - overlapping_angle) / (np.pi / 6)

        if coverage[epoch] >= 0.85:
            time = time + 1
            timeCov[epoch] = time / epoch

        for index, angle in enumerate(Angle):
            if angle < angleEnd and angle > angleStart:
                path = [
                    [circleX + r * np.cos(angle - cov/2), circleY + r * np.sin(angle - cov/2)],
                    [circleX + r * np.cos(angle + cov/2), circleY + r * np.sin(angle + cov/2)],
                    [circleX, circleY]
                ]
                plt.setp(verHandle[index], xy=path)

_, ax2 = plt.subplots()
plt.plot(lambda_h)
plt.title('代数连通度变化')

_, ax3 = plt.subplots()
plt.plot(coverage)
plt.title('覆盖率变化')


_, ax4 = plt.subplots()
plt.plot(timeCov)
plt.title('时间覆盖率')

# 防止绘图关闭
plt.ioff()
plt.show()