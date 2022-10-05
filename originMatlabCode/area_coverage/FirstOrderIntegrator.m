close all;
clear;
%% 读取场强数据和处理
Data = csvread('data.csv');
DataRange = 50;
FsyData = 10*DataTransform(Data,DataRange);
%% 连通保持控制器参数设定
R = 25; %通信半径
deta = 0.1; %通信边界处的边权大小，越小效果越好
Epsilon = 0.1; %最小代数连通度
%% 参数设置
UAVnumber = 50; %无人机数量
rng(1);
coverageRadius = 10; %无人机覆盖半径
connect = 100; %连通保持约束，每次移动不超过5
% PositionX = rand(UAVnumber,1)*20+25; %无人机初始x轴位置
% PositionY = rand(UAVnumber,1)*20+25; %无人机初始y轴位置
PositionX = rand(UAVnumber,1)*20+35; %无人机初始x轴位置
PositionY = rand(UAVnumber,1)*20+25; %无人机初始y轴位置
PisitionA = rand(UAVnumber,1); %无人机初始角速度
xrange = 100; %覆盖范围  xrange = 100; 实际范围
yrange = 100; %覆盖范围  yrange = 100;
[gridX,gridY] = meshgrid(xrange/DataRange/2:xrange/DataRange:xrange-xrange/DataRange/2,...
    yrange/DataRange/2:xrange/DataRange:yrange-yrange/DataRange/2);
subplot(2,2,3)
%meshc(gridX,gridY,FsyData);
surf(gridX,gridY,FsyData);
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
%dt = 0.1; %控制律更新间隔
dt = 0.02; %控制律更新间隔
numIterations = 5000; %迭代次数
UAVSpeed = 2; %智能体的最大速度
vmax = 4; %连通保持控制律限幅
omega = 4; %无人机绕圆的半径 UAVSpeed/omega
PathX = zeros(numIterations+1,UAVnumber); %x轨迹存储
PathY = zeros(numIterations+1,UAVnumber); %y轨迹存储
PathVirtualX = zeros(numIterations+1,UAVnumber); %x虚拟中心轨迹储存
PathVirtualY = zeros(numIterations+1,UAVnumber); %y虚拟中心轨迹储存
PathA = zeros(numIterations+1,UAVnumber); %无人机朝向轨迹存储
PathX(1,:) = PositionX';
PathY(1,:) = PositionY';
PathA(1,:) = PisitionA';
covControlX = zeros(numIterations,UAVnumber); %覆盖控制律X
covControlY = zeros(numIterations,UAVnumber); %覆盖控制律Y
connControlX = zeros(numIterations,UAVnumber); %连通控制律X
connControlY = zeros(numIterations,UAVnumber); %连通控制律Y
allControlX = zeros(numIterations,UAVnumber); %总控制率X
allControlY = zeros(numIterations,UAVnumber); %总控制率Y
% 实际位置转虚拟位置
[PxVirtual,PyVirtual] = ActualToVirtual(PathX(1,:),PathY(1,:),PathA(1,:),UAVSpeed,omega);
PathVirtualX(1,:) = PxVirtual;
PathVirtualY(1,:) = PyVirtual;
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
currHandle = plot(zeros(4,UAVnumber),zeros(4,UAVnumber),'LineWidth',1.5,'color','b');
%无人机初始位置画图
edges = [-0.15,0.32,-0.15,-0.15;-0.1,0,0.15,-0.1]*5; %三角形的大小定义
n_edges = zeros(2*UAVnumber,4);
for k = 1:UAVnumber
    Rt = [cos(PathA(1,k)), -sin(PathA(1,k)); sin(PathA(1,k)), cos(PathA(1,k))];
    for i = 1:4
        n_edges(1+2*(k-1):2*k,i) = Rt*edges(:,i) + [PathX(1,k);PathY(1,k)];
    end
end
for i=1:UAVnumber
    set(currHandle(i),'XData',real(n_edges(2*i-1,:)'),'YData',real(n_edges(2*i,:)'));
end
%% 计算初始位置覆盖率
%非均匀区域计算最优虚拟位置
[cxOPI,cyOPI] = OptimalPositionNonuniform(PathVirtualX(1,:),PathVirtualY(1,:),FsyData,gridX,gridY);
%当维诺区域较小，上述最优位置计算可能不存在,则无人机保持原位置
for k = 1:UAVnumber
    if isnan(cxOPI(1,k))
        [cxOPI(1,k),cyOPI(1,k)] = ActualToVirtual(PathX(1,k),PathY(1,k),PathA(1,k),UAVSpeed,omega);
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
axis([0,numIterations,0,5]);
plot(0:1:numIterations,zeros(1,numIterations+1)+Epsilon);

%% 算法运行
for counter = 1:numIterations
    %% 每5次迭代计算一次最优位置
    if mod(counter,1)==0
        %非均匀区域计算最优虚拟位置
        [cxOPI,cyOPI] = OptimalPositionNonuniform(PathVirtualX(counter,:),PathVirtualY(counter,:),FsyData,gridX,gridY);
        %当维诺区域较小，上述最优位置计算可能不存在,则无人机保持原位置
        for k = 1:UAVnumber
            if isnan(cxOPI(1,k))
                [cxOPI(1,k),cyOPI(1,k)] = ActualToVirtual(PathX(counter,k),PathY(counter,k),PathA(counter,k),UAVSpeed,omega);
            end
        end
    else
    end
    %约束最优位置达到限制运动目的，以此来保持连通
    %     if counter <400
    %         cx = cx+1;
    %         cy = cy+1;
    %     else
    %     end
    % [cx,cy] = ConnectConstraint(PathVirtualX(counter,:),PathVirtualY(counter,:),cx,cy,connect);  
    %% 最优位置转无人机速度方向的函数
    speedAngele = covUnicycle(PathX(counter,:),PathY(counter,:),...
        PathA(counter,:),cxOPI,cyOPI,UAVSpeed,omega);
    PathA(counter+1,:) = speedAngele;
    
    %% 无连通保持的控制律
    covControlX(counter,:) = UAVSpeed*cos(PathA(counter+1,:));
    covControlY(counter,:) = UAVSpeed*sin(PathA(counter+1,:));
      
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
    allControlX(counter,:) = covControlX(counter,:)+connControlX(counter,:);
    allControlY(counter,:) = covControlY(counter,:)+connControlY(counter,:);
    %      allControlX(counter,:) = covControlX(counter,:);
    %     allControlY(counter,:) = covControlY(counter,:);
    
    %% 最优位置 
    cx = PathX(counter,:)+dt*allControlX(counter,:);
    cy = PathY(counter,:)+dt*allControlY(counter,:);
    [cxVir,cyVir] = ActualToVirtual(cx,cy, PathA(counter,:),UAVSpeed,omega);
    PathX(counter+1,:) = cx;
    PathY(counter+1,:) = cy;
    
    %% 判断是否旋转
    distance = sqrt((PathVirtualX(counter,:)-cxVir).^2+(PathVirtualY(counter,:)-cyVir).^2);
    for k=1:UAVnumber
        %if distance(1,k) < 0.0005
        %if distance(1,k) < 0.05
        if distance(1,k) < 0.01
            if counter-1==0
                PathA(counter+1,k) = PathA(counter,k)+omega*dt;
            else
                PathA(counter+1,k) = PathA(counter,k)+sign(PathA(counter,k)-PathA(counter-1,k))*omega*dt;
            end
            PathA(counter+1,i) = mod(PathA(counter+1,i),2*pi);
            PathX(counter+1,k) = PathX(counter,k)+dt*UAVSpeed*cos(PathA(counter+1,k));
            PathY(counter+1,k) = PathY(counter,k)+dt*UAVSpeed*sin(PathA(counter+1,k));
        else
        end
    end
    
    % 约束，保证无人机不出界
    for k=1:UAVnumber
        PathX(counter+1,k) = max(PathX(counter+1,k),UAVSpeed/omega);
    end
    [PathVirtualX(counter+1,:),PathVirtualY(counter+1,:)] =...
        ActualToVirtual(PathX(counter+1,:),PathY(counter+1,:),PathA(counter+1,:),UAVSpeed,omega);
    % 画图小三角
    n_edges = zeros(2*UAVnumber,4);
    for k = 1:UAVnumber
        Rt = [cos(PathA(counter+1,k)), -sin(PathA(counter+1,k)); sin(PathA(counter+1,k)), cos(PathA(counter+1,k))];
        for i = 1:4
            n_edges(1+2*(k-1):2*k,i) = Rt*edges(:,i) + [PathX(counter+1,k);PathY(counter+1,k)];
        end
    end
    % 输出无人机位置
    for i=1:UAVnumber
        set(currHandle(i),'XData',real(n_edges(2*i-1,:)'),'YData',real(n_edges(2*i,:)'));
    end
    %输出维诺划分
    [v,c]=VoronoiBounded(PathVirtualX(counter+1,:)',PathVirtualY(counter+1,:)', crs);
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