function [] = coverageTime(UAVnumber,R,coverageRadius)
% close all;
% clear;
%覆盖时间对空间覆盖率的影响
% 一张figure，2*2  左上位置+背景；右上连通度变化曲线；左下时间覆盖率；右下空间覆盖率
%% 读取场强数据和处理
Data = readmatrix('dataLT.csv');
DataRange = 50;
FsyData = 10000000*DataTransform(Data,DataRange);
FsyData = FsyData+0.05;
%% 连通保持控制器参数设定
%R = 25; %通信半径
deta = 0.1; %通信边界处的边权大小，越小效果越好
Epsilon = 0.1; %最小代数连通度
warnEpsilon = 1; %连通度警戒值
%% 参数设置
%UAVnumber = 60; %无人机数量
%numIterations = 1000; %迭代次数
numIterations = 2500; %迭代次数
%rng(2); 默认
%rng(3);
%coverageRadius = 5; %无人机覆盖半径
connect = 2; %限制了每次无人机的移动距离
dt = 0.1; %控制律更新间隔
vmax = 2; %连通保持控制律限幅
% PositionX = rand(UAVnumber,1)*20+75; %无人机初始x轴位置
% PositionY = rand(UAVnumber,1)*20+5; %无人机初始y轴位置

PositionX = rand(UAVnumber,1)*20+60; %无人机初始x轴位置
PositionY = rand(UAVnumber,1)*20+55; %无人机初始y轴位置

% 创建飞机
xrange = 100; %覆盖范围  xrange = 100; 实际范围
yrange = 100; %覆盖范围  yrange = 100;
[gridX,gridY] = meshgrid(xrange/DataRange/2:xrange/DataRange:xrange-xrange/DataRange/2,...
    yrange/DataRange/2:xrange/DataRange:yrange-yrange/DataRange/2);
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

%% 计算空间覆盖率
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

%% 计算时间覆盖率
coverageTime = 0; %离散点是否被覆盖
if coverageRate(1,1)>=0.85
   coverageTime =  coverageTime+1;
else 
end
coverageTimeRate(1,1) = coverageTime/1;

%% 初始连通度计算
x = [PathX(1,:)' PathY(1,:)'];
[L,A,d] = LaplaMat(x,R,deta);
[vector,value]=eig(L);
lamde_h(1,1) = value(2,2);
[cxOPI,cyOPI] = OptimalPositionNonuniform(PathX(1,:),PathY(1,:),FsyData,gridX,gridY);
%当维诺区域较小，上述最优位置计算可能不存在,则无人机保持原位置
for k = 1:UAVnumber
    if isnan(cxOPI(1,k))
        cxOPI(1,k) = PathX(1,k);
        cyOPI(1,k) = PathY(1,k);
    end
end

%% 算法运行
for counter = 1:numIterations
[cxOPI,cyOPI] = OptimalPositionNonuniform(PathX(counter,:),PathY(counter,:),...
    FsyData,gridX,gridY);
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
    else %警戒范围外，不进行连通保持控制
        allControlX(counter,agent) = covControlX(counter,agent);
        allControlY(counter,agent) = covControlY(counter,agent);
    end
end
allControlX(counter,:) = covControlX(counter,:) + connControlX(counter,:);
allControlY(counter,:) = covControlY(counter,:) + connControlY(counter,:);
     
%% 最优位置
PathX(counter+1,:) = PathX(counter,:)+dt*allControlX(counter,:);
PathY(counter+1,:) = PathY(counter,:)+dt*allControlY(counter,:);

%% 判断是否移动
distance = sqrt((PathX(counter+1,:)-PathX(counter,:)).^2+(PathY(counter+1,:)-PathY(counter,:)).^2);
for k=1:UAVnumber
%if distance(1,k) < 0.0005
    if distance(1,k) < 0.01
        PathX(counter+1,k) = PathX(counter,k);
        PathY(counter+1,k) = PathY(counter,k);
    else
    end
end

%% 空间覆盖率
coverageState = 0;  %被覆盖的离散点的个数
for j = 1:DataRange  %通过计算距离，将点分配给维诺划分
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

%% 时间覆盖率
if coverageRate(1,counter+1)>=0.85
    coverageTime =  coverageTime+1;
else
end
coverageTimeRate(1,counter+1) = coverageTime/(counter+1);
%set(coverageTimeHandle,'XData',1:counter+1,'YData',coverageTimeRate);

%% 连通度计算
x = [PathX(counter+1,:)' PathY(counter+1,:)'];
[L,A,d] = LaplaMat(x,R,deta);
[vector,value]=eig(L);
lamde_h(1,counter+1) = value(2,2);
end
%% 指标计算
[first,second]=find(coverageTimeRate>=0.85);
timaRateTime=min(second);

[firstspa,secondspa]=find(coverageRate>=0.85);
spaceRateTime=min(secondspa);

fprintf('无人机数量 %8.5f\n',UAVnumber);
fprintf('覆盖半径 %8.5f\n',coverageRadius);
fprintf('通信半径 %8.5f\n',R);
fprintf('算法结束时间覆盖率 %8.5f\n',coverageTimeRate(1,end));
fprintf('完成时间覆盖率迭代次数 %8.5f\n',timaRateTime);
fprintf('算法结束空间覆盖率 %8.5f\n',coverageRate(1,end));
fprintf('完成空间覆盖率迭代次数 %8.5f\n',spaceRateTime);
if min(lamde_h)>0
    fprintf('保持连通 %8.5f\n');
else
     fprintf('连通断开 %8.5f\n');
end
fprintf('最小代数连通度 %8.5f\n',min(lamde_h));
end