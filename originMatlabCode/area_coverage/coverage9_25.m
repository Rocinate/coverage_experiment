close all;
clear;
% 一张figure，2*2  左上位置+背景；右上覆盖范围；左下拓扑；右下覆盖率  
%% 读取场强数据和处理
fileID = fopen('scene2.txt','w');
Data = readmatrix('dataLT.csv');
DataRange = 50;
FsyData = 10000000*DataTransform(Data,DataRange);
FsyData = FsyData+0.05;
%% 连通保持控制器参数设定
R = 25; %通信半径
deta = 0.1; %通信边界处的边权大小，越小效果越好
Epsilon = 0.1; %最小代数连通度
warnEpsilon = 1; %连通度警戒值
%% 参数设置
UAVnumber = 60; %无人机数量
numIterations = 1000; %迭代次数
%numIterations = 2500; %迭代次数
rng(2);
coverageRadius = 10; %无人机覆盖半径
connect = 2; %限制了每次无人机的移动距离
dt = 0.1; %控制律更新间隔
vmax = 2; %连通保持控制律限幅
PositionX = rand(UAVnumber,1)*20+75; %无人机初始x轴位置
PositionY = rand(UAVnumber,1)*20+5; %无人机初始y轴位置
% 创建飞机
s = creatPlaneJson([PositionX,PositionY]);
string = jsonencode(s);
string = unicode2native(string, 'UTF-8');
fprintf(fileID,"%s\n",string);
xrange = 100; %覆盖范围  xrange = 100; 实际范围
yrange = 100; %覆盖范围  yrange = 100;
[gridX,gridY] = meshgrid(xrange/DataRange/2:xrange/DataRange:xrange-xrange/DataRange/2,...
    yrange/DataRange/2:xrange/DataRange:yrange-yrange/DataRange/2);
%figure(1)
subplot(2,2,1)
%meshc(gridX,gridY,FsyData);
%s = surf(gridX,gridY,FsyData);
[gridXX,gridYY] = meshgrid(0:0.1:99.9,0:0.1:99.9);
s = pcolor(gridXX,gridYY,Data*100000000);
s.EdgeColor = 'none';
%s = surf(0:0.1:99.9,0:0.1:99.9,Data);
hold on
%s.EdgeColor = 'none';
view(2);
% title('任务区域场强图')
% xlabel('x')
% ylabel('y')
crs = [0,0; 0, yrange; xrange, yrange; xrange, 0]; %覆盖区域
 x_range=[0, xrange, xrange, 0, 0];
 y_range=[0, 0, yrange, yrange, 0];
hold on;
% subplot(2,2,1)
%figure(1)
hold on
plot(x_range,y_range); %覆盖区域画图
title('无人机轨迹')
xlabel('x')
ylabel('y')
%axis equal
axis([0,xrange,0,yrange]);
hold on;
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
[v,c]=VoronoiBounded(PathX(1,:)',PathY(1,:)', crs);
for i = 1:UAVnumber  %维诺划分的设置
    %verCellHandle(i) = patch(v(c{i},1),v(c{i},2),cellColors(i,:),'LineWidth',1);
    %verCellHandle(i) = patch(v(c{i},1),v(c{i},2),'cellColors(i,:)','FaceColor','none');
    verCellHandle(i) = patch(v(c{i},1),v(c{i},2),'w','LineWidth',1,'EdgeColor','w','FaceColor','none');
    hold on
end
%无人机轨迹
% pathHandle = rand(UAVnumber,1);
% for i = 1:UAVnumber % 无人机轨迹的设置
%     pathHandle(i)  = plot(Px(i),Py(i),'-','color',cellColors(i,:)*.8);
% end
%currHandle = plot(Px,Py,'o','linewidth',2);
%% 无人机当前位置画图
currHandle = plot(PathX(1,:),PathY(1,:),'o','LineWidth',1,'color','w');
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
%% 覆盖范围画图
subplot(2,2,2)
CoverageRangeHandle = rand(UAVnumber,1); %维诺划分作图设置
for i = 1:UAVnumber 
    CoverageRangeHandle(i) = rectangle('Position',[PathX(1,i)-coverageRadius PathY(1,i)-coverageRadius 2*coverageRadius 2*coverageRadius],'Curvature',[1,1],'FaceColor','none','EdgeColor','b');
    %CoverageRangeHandle(i) = viscircles([PathX(1,i) PathY(1,i)], coverageRadius,'FaceColor','k');
    hold on
end
axis([0,xrange,0,yrange]);
currHandle2 = plot(PathX(1,:),PathY(1,:),'o','LineWidth',1,'color','b');

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
subplot(2,2,4)
%figure(3)
coverageHandle = plot(0,0,'-','LineWidth',1,'color','r');
title('空间覆盖率变化曲线')
xlabel('迭代次数')
ylabel('覆盖率')
axis([0,numIterations,0,1]);
hold on;
plot(0:1:numIterations,zeros(1,numIterations+1)+0.85,'--','color','b');


%% 计算时间覆盖率
% coverageTime = 0; %离散点是否被覆盖
% if coverageRate(1,1)>=0.85
%    coverageTime =  coverageTime+1;
% else 
% end
% coverageTimeRate(1,1) = coverageTime/1;
% %subplot(2,2,3)
% figure(5)
% coverageTimeHandle = plot(0,0,'-','LineWidth',1,'color','r');
% title('时间覆盖率变化曲线')
% xlabel('迭代次数')
% ylabel('覆盖率')
% axis([0,numIterations,0,1]);
% hold on;
% plot(0:1:numIterations,zeros(1,numIterations+1)+0.85,'--','color','b');

%% 初始连通度计算
x = [PathX(1,:)' PathY(1,:)'];
[L,A,d] = LaplaMat(x,R,deta);
[vector,value]=eig(L);
lamde_h(1,1) = value(2,2);
%subplot(2,2,4)
% figure(4)
% lamdaHandle = plot(0,0,'-','LineWidth',1,'color','r');
% hold on;
% title('连通度变化曲线')
% xlabel('迭代次数')
% ylabel('连通度')
% axis([0,numIterations,0,30]);
% plot(0:1:numIterations,zeros(1,numIterations+1)+Epsilon,'--','color','b');

[cxOPI,cyOPI] = OptimalPositionNonuniform(PathX(1,:),PathY(1,:),FsyData,gridX,gridY);
%当维诺区域较小，上述最优位置计算可能不存在,则无人机保持原位置
for k = 1:UAVnumber
    if isnan(cxOPI(1,k))
        cxOPI(1,k) = PathX(1,k);
        cyOPI(1,k) = PathY(1,k);
    end
end
curves = [coverageRate(1,1),lamde_h(1,1)];
s = creatScene2Json(x,zeros(UAVnumber,2),A,[cxOPI',cyOPI'],v,c,curves,dt);
string = jsonencode(s);
string = unicode2native(string, 'UTF-8');
fprintf(fileID,"%s\n",string);
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
    allControlX(counter,:) = covControlX(counter,:);
    allControlY(counter,:) = covControlY(counter,:);
    PathX(counter+1,:) = PathX(counter,:)+dt*allControlX(counter,:);
    PathY(counter+1,:) = PathY(counter,:)+dt*allControlY(counter,:);
    allControl = [allControlX(counter,:)',allControlY(counter,:)'];
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
    %输出无人机位置
    set(currHandle,'XData',PathX(counter+1,:),'YData',PathY(counter+1,:));
    %输出维诺划分
    [v,c]=VoronoiBounded(PathX(counter+1,:)',PathY(counter+1,:)', crs);
    for i = 1:numel(c) % 更新维诺划分
        set(verCellHandle(i), 'XData',v(c{i},1),'YData',v(c{i},2));
    end
    
    %% 覆盖范围

for i = 1:UAVnumber 
    set(CoverageRangeHandle(i),'Position',[PathX(counter+1,i)-coverageRadius PathY(counter+1,i)-coverageRadius coverageRadius*2 coverageRadius*2],'Curvature',[1,1]);
    %CoverageRangeHandle(i) = viscircles([PathX(1,i) PathY(1,i)], coverageRadius,'FaceColor','k');
end
set(currHandle2,'XData',PathX(counter+1,:),'YData',PathY(counter+1,:));

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
    set(coverageHandle,'XData',1:counter+1,'YData',coverageRate);
    
%     %% 时间覆盖率
%     if coverageRate(1,counter+1)>=0.85
%         coverageTime =  coverageTime+1;
%     else
%     end
%     coverageTimeRate(1,counter+1) = coverageTime/(counter+1);
%     set(coverageTimeHandle,'XData',1:counter+1,'YData',coverageTimeRate);
    
    
    %% 连通度计算
    x = [PathX(counter+1,:)' PathY(counter+1,:)'];
    [L,A,d] = LaplaMat(x,R,deta);
    [vector,value]=eig(L);
    lamde_h(1,counter+1) = value(2,2);
%     set(lamdaHandle,'XData',1:counter+1,'YData',lamde_h);
     drawnow
    curves = [coverageRate(1,counter+1),lamde_h(1,counter+1)];
    s = creatScene2Json(x,allControl,A,[cxOPI',cyOPI'],v,c,curves,dt);
    string = jsonencode(s);
    string = unicode2native(string, 'UTF-8');
    fprintf(fileID,"%s\n",string);
end
%% 计算小框内的连通度
BoxCenter = [27,29.5;51.5,59;69.5,80;71,29];
BoxRadius = [14,14;9,10;7,8;9,9.5];
for k =1:size(BoxCenter,1)
    BoxPositionX = [];
    BoxPositionY = [];
    numberInside = 0;
    for j = 1:UAVnumber
        BoxX =[BoxCenter(k,1)-BoxRadius(k,1), BoxCenter(k,1)+BoxRadius(k,1),...
            BoxCenter(k,1)+BoxRadius(k,1),BoxCenter(k,1)-BoxRadius(k,1),BoxCenter(k,1)-BoxRadius(k,1)];
        BoxY =[BoxCenter(k,2)-BoxRadius(k,2), BoxCenter(k,2)-BoxRadius(k,2),...
            BoxCenter(k,2)+BoxRadius(k,2),BoxCenter(k,2)+BoxRadius(k,2),BoxCenter(k,2)-BoxRadius(k,2)];
         figure(1)
         plot(BoxX,BoxY);
         hold on;
%         subplot(2,2,2)
%         plot(BoxX,BoxY);
%         hold on
        [in,on] = inpolygon(PathX(numIterations+1,j),PathY(numIterations+1,j),BoxX,BoxY);
        if in ==1
            numberInside = numberInside+1;
            BoxPositionX(numberInside,1) = PathX(numIterations+1,j);
            BoxPositionY(numberInside,1) = PathY(numIterations+1,j);
        else
        end
    end
    
    x = [BoxPositionX BoxPositionY];
    [L,A,d] = LaplaMat(x,R,deta);
    [vector,value]=eig(L);
    for m=1:10*R
        if value(2,2)>0
            Rt = R-m*0.1;
            [L,A,d] = LaplaMat(x,Rt,deta);
            [vector,value]=eig(L);
        else
            break;
        end
        NewRang(k) = Rt+1;  
    end
end
 NewRang

%  %% 改变覆盖半径，计算空间覆盖率
%  coverageState = 0;  %被覆盖的离散点的个数
%  for j = 1:DataRange  %通过计算距离，将点分配给维诺划分
%      for k = 1:DataRange
%          minDistan = min(sqrt((PathX(counter+1,:)-(gridX(j,k)+zeros(1,UAVnumber))).^2+...
%              (PathY(counter+1,:)-(gridY(j,k)+zeros(1,UAVnumber))).^2));
%          if minDistan <= coverageRadius
%              coverageState = coverageState+FsyData(j,k);
%          else
%          end
%      end
%  end
%  coverageChangeRate(1,1) = coverageState/coverageAll;
%  
%  
%  for m = 1:coverageRadius*10
%      coverageState = 0;  %被覆盖的离散点的个数
%      NewCoverageRadius = coverageRadius-m*0.1;
%      
%      for j = 1:DataRange  %通过计算距离，将点分配给维诺划分
%          for k = 1:DataRange
%              minDistan = min(sqrt((PathX(counter+1,:)-(gridX(j,k)+zeros(1,UAVnumber))).^2+...
%                  (PathY(counter+1,:)-(gridY(j,k)+zeros(1,UAVnumber))).^2));
%              if minDistan <= NewCoverageRadius
%                  coverageState = coverageState+FsyData(j,k);
%              else
%              end
%          end
%      end
%      coverageChangeRate(m+1,1)= coverageState/coverageAll;
%      coverageChangeRate(m+1,2)= NewCoverageRadius;
%      
%  end
 
 
 fclose(fileID);
 fclose('all');
 
 
 