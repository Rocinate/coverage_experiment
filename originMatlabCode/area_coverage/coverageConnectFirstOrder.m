close all;
clear;
%% 读取场强数据和处理
Data = csvread('data.csv');
DataRange = 50;
FsyData = 10000*DataTransform(Data,DataRange);
FsyData = FsyData + 0.1;
%% 连通保持控制器参数设定
R = 25; %通信半径
deta = 0.1; %通信边界处的边权大小，越小效果越好
Epsilon = 0.1; %最小代数连通度
warnEpsilon = 1; %连通度警戒值
%% 参数设置
UAVnumber = 50; %无人机数量
rng(1);
coverageRadius = 10; %无人机覆盖半径
connect = 2; %连通保持约束，每次移动不超过5
% PositionX = rand(UAVnumber,1)*20+25; %无人机初始x轴位置
% PositionY = rand(UAVnumber,1)*20+25; %无人机初始y轴位置
PositionX = rand(UAVnumber,1)*20+5; %无人机初始x轴位置
PositionY = rand(UAVnumber,1)*20+65; %无人机初始y轴位置
xrange = 100; %覆盖范围  xrange = 100; 实际范围
yrange = 100; %覆盖范围  yrange = 100;
[gridX,gridY] = meshgrid(xrange/DataRange/2:xrange/DataRange:xrange-xrange/DataRange/2,...
    yrange/DataRange/2:xrange/DataRange:yrange-yrange/DataRange/2);
subplot(2,2,3)
%meshc(gridX,gridY,FsyData);
s = surf(gridX,gridY,FsyData);
s.EdgeColor = 'none';
view(2);
title('任务区域场强图')
xlabel('x')
ylabel('y')
crs = [0,0; 0, yrange; xrange, yrange; xrange, 0]; %覆盖区域
x_range=[0, xrange, xrange, 0, 0];
y_range=[0, 0, yrange, yrange, 0];
subplot(2,2,1)
%figure(1)
hold on
plot(x_range,y_range); %覆盖区域画图
title('无人机轨迹')
xlabel('x')
ylabel('y')
axis equal
axis([0,xrange,0,yrange]);
hold on;
%dt = 1; %控制律更新间隔
dt = 0.1; %控制律更新间隔
numIterations = 2000; %迭代次数
UAVSpeed = 2; %智能体的最大速度
vmax = 2; %连通保持控制律限幅
PathX = zeros(numIterations+1,UAVnumber); %x轨迹存储
PathY = zeros(numIterations+1,UAVnumber); %y轨迹存储
PathX(1,:) = PositionX';
PathY(1,:) = PositionY';
covControlX = zeros(numIterations,UAVnumber); %覆盖控制律X
covControlY = zeros(numIterations,UAVnumber); %覆盖控制律Y
connControlX = zeros(numIterations,UAVnumber); %连通控制律X
connControlY = zeros(numIterations,UAVnumber); %连通控制律Y
allControlX = zeros(numIterations,UAVnumber); %总控制率X
allControlY = zeros(numIterations,UAVnumber); %总控制率Y
%% 画图设置
verCellHandle = rand(UAVnumber,1); %维诺划分作图设置
%cellColors = cool(n); %维诺划分是否添加色彩
cellColors = zeros(UAVnumber,3)+1; %维诺划分颜色置为白色
for i = 1:UAVnumber  %维诺划分的设置
    verCellHandle(i)  = patch(PositionX(i),PositionY(i),cellColors(i,:));
    hold on
end
%无人机轨迹
% pathHandle = rand(UAVnumber,1);
% for i = 1:UAVnumber % 无人机轨迹的设置
%     pathHandle(i)  = plot(Px(i),Py(i),'-','color',cellColors(i,:)*.8);
% end
%currHandle = plot(Px,Py,'o','linewidth',2);
%% 无人机当前位置画图
currHandle = plot(PathX(1,:),PathY(1,:),'o','LineWidth',1,'color','b');
%% 计算初始位置覆盖率
%非均匀区域计算最优虚拟位置
[cxOPI,cyOPI] = OptimalPositionNonuniform(PathX(1,:),PathY(1,:),FsyData,gridX,gridY);
%当维诺区域较小，上述最优位置计算可能不存在,则无人机保持原位置
for k = 1:UAVnumber
    if isnan(cxOPI(1,k))
        cxOPI(1,k) = PathX(1,k);
        cyOPI(1,k) = PathY(1,k);
    end
end
%% 计算覆盖率
coverageState = 0; %离散点是否被覆盖
coverageAll = sum(FsyData(:)); %总场强
for j = 1:DataRange %通过计算距离，将点分配给维诺划分
    for k = 1:DataRange
        minDistan = min(sqrt((PathX(1,:)-(gridX(j,k)+zeros(1,UAVnumber))).^2+...
            (PathY(1,:)-(gridY(j,k)+zeros(1,UAVnumber))).^2));
        if minDistan <= coverageRadius
            coverageState = coverageState+FsyData(j,k);
        else
        end
    end
end
coverageRate(1,1) = coverageState/coverageAll;
subplot(2,2,2)
%figure(2)
coverageHandle = plot(0,0,'-','LineWidth',1,'color','r');
title('覆盖率变化曲线')
xlabel('迭代次数')
ylabel('覆盖率')
axis([0,numIterations,0,1]);

%% 初始连通度计算
x = [PathX(1,:)' PathY(1,:)'];
[L,A,d] = LaplaMat(x,R,deta);
[vector,value]=eig(L);
lamde_h(1,1) = value(2,2);
subplot(2,2,4)
lamdaHandle = plot(0,0,'-','LineWidth',1,'color','r');
hold on;
title('连通度变化曲线')
xlabel('迭代次数')
ylabel('连通度')
axis([0,numIterations,0,22]);
plot(0:1:numIterations,zeros(1,numIterations+1)+Epsilon);

%% 算法运行
for counter = 1:numIterations
    [cxOPI,cyOPI] = OptimalPositionNonuniform(PathX(counter,:),PathY(counter,:),FsyData,gridX,gridY);
    %当维诺区域较小，上述最优位置计算可能不存在,则无人机保持原位置
    for k = 1:UAVnumber
        if isnan(cxOPI(1,k))
            cxOPI(1,k) = PathX(counter,k);
            cyOPI(1,k) = PathY(counter,k);
        end
    end
    %% 覆盖控制律
    [covUx,covUy] = covO3(PathX(counter,:),PathY(counter,:),cxOPI,cyOPI,connect);
    
    %% 无连通保持的控制律
    covControlX(counter,:) = covUx;
    covControlY(counter,:) = covUy;
    
    %% 连通保持控制律
    lamde2est=ones(UAVnumber,1)*value(2,2);
    v2=vector(:,2);
    uc=connect_preserve(lamde2est,v2,x,d,A,R,deta,Epsilon);
    
    for agent=1:UAVnumber
        if norm(uc(agent,:))>vmax
            uc(agent,:)=vmax*uc(agent,:)./norm(uc(agent,:));
        end
    end
    connControlX(counter,:) = uc(:,1)';
    connControlY(counter,:) = uc(:,2)';
    
    %% 总控制
    critical=is_critical_robot(d,R,0.7);
    for agent=1:UAVnumber
        if lamde2est(agent)<=warnEpsilon
            if critical(agent)==1 % 关键机器人保留全部连通保持控制
                allControlX(counter,agent) = covControlX(counter,agent)+connControlX(counter,agent);
                allControlY(counter,agent) = covControlY(counter,agent)+connControlY(counter,agent);
            else%非关键机器人保留部分连通保持控制
                allControlX(counter,agent) = covControlX(counter,agent)+0.8*connControlX(counter,agent);
                allControlY(counter,agent) = covControlY(counter,agent)+0.8*connControlY(counter,agent);
            end
        else%警戒范围外，不进行连通保持控制
            allControlX(counter,agent) = covControlX(counter,agent);
            allControlY(counter,agent) = covControlY(counter,agent);
        end
    end
%     allControlX(counter,:) = covControlX(counter,:) + connControlX(counter,:);
%     allControlY(counter,:) = covControlY(counter,:) + connControlY(counter,:);
    %% 最优位置
    PathX(counter+1,:) = PathX(counter,:)+dt*allControlX(counter,:);
    PathY(counter+1,:) = PathY(counter,:)+dt*allControlY(counter,:);
    
    %% 判断是否移动
         distance = sqrt((PathX(counter+1,:)-PathX(counter,:)).^2+(PathY(counter+1,:)-PathY(counter,:)).^2);
         for k=1:UAVnumber
            %if distance(1,k) < 0.0005
            %if distance(1,k) < 0.05
            if distance(1,k) < 0.01
    
                PathX(counter+1,k) = PathX(counter,k);
                PathY(counter+1,k) = PathY(counter,k);
            else
            end
        end
    %输出无人机位置
    set(currHandle,'XData',PathX(counter+1,:),'YData',PathY(counter+1,:));
    
    %输出维诺划分
    [v,c]=VoronoiBounded(PathX(counter+1,:)',PathY(counter+1,:)', crs);
    for i = 1:numel(c) % 更新维诺划分
        set(verCellHandle(i), 'XData',v(c{i},1),'YData',v(c{i},2));
    end
    %% 覆盖率
    coverageState = 0; %离散点是否被覆盖
    for j = 1:DataRange %通过计算距离，将点分配给维诺划分
        for k = 1:DataRange
            minDistan = min(sqrt((PathX(counter+1,:)-(gridX(j,k)+zeros(1,UAVnumber))).^2+...
                (PathY(counter+1,:)-(gridY(j,k)+zeros(1,UAVnumber))).^2));
            if minDistan <= coverageRadius
                coverageState = coverageState+FsyData(j,k);
            else
            end
        end
    end
    coverageRate(1,counter+1) = coverageState/coverageAll;
    set(coverageHandle,'XData',1:counter+1,'YData',coverageRate);
    %% 连通度计算
    x = [PathX(counter+1,:)' PathY(counter+1,:)'];
    [L,A,d] = LaplaMat(x,R,deta);
    [vector,value]=eig(L);
    lamde_h(1,counter+1) = value(2,2);
    set(lamdaHandle,'XData',1:counter+1,'YData',lamde_h);
    drawnow
end