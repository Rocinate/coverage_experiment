close all;
clear all;
%% 覆盖扇面作图
figure(1)
r = 3000; %雷达半径
circlex = 21000; circley = 5500; %雷达中心
anglestar = pi*165/180; angleend = pi*195/180; %扇面覆盖范围30°
cov = 2/180*pi; %单机覆盖角度
qw = 100; %计算覆盖扇面，用于画图，划分精度
qwe = anglestar:(angleend-anglestar)/qw:angleend; %计算覆盖扇面位置,用于作图
for i = 1:qw
    xx(i) = circlex+r*cos(qwe(i));  yy(i) = circley+r*sin(qwe(i));
end
xxx = [circlex,xx,circlex];  yyy = [circley,yy,circley];
figure(1);
plot(xxx,yyy,'-'); %axis off%覆盖扇面作图
%fill(xxx,yyy,'r');
axis([-5000,21000,-5000,18000]); %画图范围
hold on;
%% 无人机初始状态设置
n = 35; %每一批次无人机数量
sd = 2;  rng(sd);
for i = 1:n
    Px(i,1) = 3500+100*rand(1,1)-i*25000/n;
end
%Px(1:n,1) = rand(n,1)*20000-20000;
Py(1:n,1) = rand(n,1)*3000+4000; %第一批次无人机位置


% 无人机开始执行覆盖任务和结束覆盖任务区域
plot(zeros(1,1001)+1000,-5000:23000/1000:18000,'--');
plot(zeros(1,1001)+18000,-5000:23000/1000:18000,'--');
% 无人机初始角度
Angle = pi+atan((circley-Py)./(circlex-Px));
%% 作图设置
%无人机覆盖时扇面颜色设置
cellColors = zeros(n,3);
for counter = 1:n
    cellColors(counter,:)=[49/255,198/255,159/255];
end
%无人机轨迹设置
pathHandle = zeros(n,1);
%currHandle = plot(Px,Py,'>','linewidth',2);
currHandle = plot(zeros(4,n),zeros(4,n),'LineWidth',1.5,'color','b');

title('无人机轨迹');
%无人机覆盖扇面作图设置
verCellHandle = zeros(n,3);
for counter = 1:n
    verCellHandle(counter)=patch(zeros(n,1),zeros(n,1),cellColors(counter,:));
end
%% 参数设置
% 连通保持控制器参数设定
R = 5000; %通信半径
deta = 0.1; %通信边界处的边权大小，越小效果越好
Epsilon = 0.05; %最小代数连通度
vmax = 60; %连通保持最大速度（用于限幅）
vx = zeros(n,1)+vmax; %无人机x轴速度
vy = zeros(n,1); %无人机y轴速度
veangle = zeros(n,1);
totalTime = 2000; %仿真总时长
dt = 0.5; %控制器更新频率
numIterations = ceil(totalTime/dt)+1; %迭代次数
t = 0;
%无人机位置、角度数据储存
t_h = zeros(numIterations,1);
Px_h = zeros(n,numIterations);
Py_h = zeros(n,numIterations);
Angle_h = zeros(n,numIterations);
Px_h(:,1) = Px;
Py_h(:,1) = Py;
Angle_h(:,1) = Angle;
%日志向量定义（记录控制量）
ue_hx = zeros(n,numIterations);
ue_hy = zeros(n,numIterations);
ue_hx(:,1) = zeros(n,1)+vmax;
ue_hy(:,1) = zeros(n,1);
uc_hx = zeros(n,numIterations);
uc_hy = zeros(n,numIterations);
u_hx = zeros(n,numIterations);
u_hy = zeros(n,numIterations);
u_hx(:,1) = ue_hx(:,1)+uc_hx(:,1);
u_hy(:,1) = ue_hy(:,1)+uc_hy(:,1);
veangle_h = zeros(n,numIterations);
veangle_h(:,1) = veangle;
%初始损失和连通度计算

x = [Px(:,1) Py(:,1)];

%%%

[L,A,d] = LaplaMat(x,R,deta);
[vector,value]=eig(L);
loss_h=zeros(numIterations,1);
lamde_h(1,1) = value(2,2);

disp([num2str(t),'时刻的连通度为',num2str(value(2,2))])
number = 0; %算法执行次数1

figure(2)
subplot(1,3,1)
plot(1:1:totalTime,zeros(1,totalTime)+Epsilon,'--','LineWidth',1.5,'color','b');
hold on;
title('代数连通度');
lamdaHandle  = plot(0,0,'-','LineWidth',1.5,'color','r');
axis([0,totalTime,0,3]);

subplot(1,3,2)
plot(1:1:totalTime,zeros(1,totalTime)+0.85,'--','LineWidth',1.5,'color','b');
hold on;
title('空间覆盖率');
coverageHandle  = plot(0,0,'-','LineWidth',1.5,'color','r');
axis([0,totalTime,0,1.1]);

subplot(1,3,3)
plot(1:1:totalTime,zeros(1,totalTime)+0.85,'--','LineWidth',1.5,'color','b');
hold on;
time_coverageHandle  = plot(0,0,'-','LineWidth',1.5,'color','r');
title('时间覆盖率')
axis([0,totalTime,0,1.1]);

shijian = 0;
firstCoverage = zeros(n,1);

for counter=1:totalTime
    %% 判断无人机是否执行覆盖任务
    coverageIndicator = zeros(n,1)+1; %判断无人机是否参与覆盖，参与赋值1，不参与赋值0
    for i=1:n
        if (Px_h(i,counter)>=1000 & Px_h(i,counter) <=18000 & u_hx(i,counter)>0)
            coverageIndicator(i,1) = 1;
            firstCoverage(i,1) = 1;
        else
            coverageIndicator(i,1) = 0;
        end
    end
    
    edges = [-150,320,-150,-150;-100,0,150,-100];
    n_edges = zeros(2*n,4);
    for k = 1:n
        Rt = [cos(veangle_h(k,counter)), -sin(veangle_h(k,counter)); sin(veangle_h(k,counter)), cos(veangle_h(k,counter))];
        for i = 1:4
            n_edges(1+2*(k-1):2*k,i) = Rt*edges(:,i) + [Px_h(k,counter);Py_h(k,counter)];
        end
    end
    
    for i=1:n
        set(currHandle(i),'XData',real(n_edges(2*i-1,:)'),'YData',real(n_edges(2*i,:)'));
        hold on;
    end
    
    %% 根据无人机是否执行覆盖任务，进行变量处理
    xpoi = Px_h(1:n,counter).*coverageIndicator;
    ypoi = Py_h(1:n,counter).*coverageIndicator;
    noCoverageIndex = find (xpoi(:,1)==0);
    noCoverageNumber =size(noCoverageIndex,1);
    uavAngle = Angle_h(:,counter);
    yCoverageLaw = ue_hy(:,counter);
    uavSpeedAngle = veangle_h(:,counter);
    coverageControlLaw = zeros(n-noCoverageNumber,1); %执行覆盖任务的y轴方向的控制律
    
    
    for k=1:noCoverageNumber
        xpoi(noCoverageIndex(noCoverageNumber+1-k),:) = [];
        ypoi(noCoverageIndex(noCoverageNumber+1-k),:) = [];
        uavAngle(noCoverageIndex(noCoverageNumber+1-k),:) = [];
        yCoverageLaw(noCoverageIndex(noCoverageNumber+1-k),:) = [];
        uavSpeedAngle(noCoverageIndex(noCoverageNumber+1-k),:) = [];
    end
    
    
    %% 角度覆盖控制律
    ue = newAngleCoverage(xpoi,ypoi,uavAngle,yCoverageLaw,uavSpeedAngle,anglestar,angleend,circlex,circley,vmax,cov,R);
    
    %% 连通约束控制
    x = [xpoi ypoi];
    [L,A,d] = LaplaMat(x,R,deta);
    [vector,value]=eig(L);
    lamde2est=ones(n-noCoverageNumber,1)*value(2,2);
    v2=vector(:,2);
    uc = connect_preserve(lamde2est,v2,x,d,A,R,deta,Epsilon);
    lamde_h(counter,1) = value(2,2);
    disp([num2str(t),'时刻的连通度为',num2str(value(2,2))])
    uc(:,2) = 10*uc(:,2);
    %限幅处理
    for agent=1:n-noCoverageNumber
        if norm(uc(agent,:))>vmax
            uc(agent,:)=vmax*uc(agent,:)./norm(uc(agent,:));
        end
    end
    
  
    %% 总控制
    for i=1:n-noCoverageNumber
        if ue(i)*(uc(i,2)+ue(i))<0
            coverageControlLaw(i) = ue(i);
        else coverageControlLaw(i) = uc(i,2)+ue(i);
        end
    end
      
    uexSpeed = zeros(n-noCoverageNumber,1)+vmax;
    
    for j = 1:noCoverageNumber
        ue = [ue(1:(noCoverageIndex(j,1)-1),1);0;ue(noCoverageIndex(j,1):end,1)];
        uc = [uc(1:(noCoverageIndex(j,1)-1),:);[0,0];uc(noCoverageIndex(j,1):end,:)];
        uexSpeed = [uexSpeed(1:(noCoverageIndex(j,1)-1),1);0;uexSpeed(noCoverageIndex(j,1):end,1)];
        coverageControlLaw = [coverageControlLaw(1:(noCoverageIndex(j,1)-1),1);0;coverageControlLaw(noCoverageIndex(j,1):end,1)];
    end
    
    ue_hx(:,counter+1) = uexSpeed;
    ue_hy(:,counter+1) = ue;
    uc_hx(:,counter+1) = uc(:,1);
    uc_hy(:,counter+1) = uc(:,2);
    u_hx(:,counter+1) = uexSpeed;
    u_hy(:,counter+1) = coverageControlLaw;
    
    %% 判断无人机执行覆盖还是返航任务
    returnSpeed = 3.5*vmax;
    for i=1:noCoverageNumber
        if Px_h(noCoverageIndex(i,1),counter)>=1000 & Px_h(noCoverageIndex(i,1),counter)<=18000
            u_hx(noCoverageIndex(i,1),counter+1) = -returnSpeed;
            u_hy(noCoverageIndex(i,1),counter+1) = 0;
        else if Px_h(noCoverageIndex(i,1),counter)<1000
                if firstCoverage(noCoverageIndex(i,1),1) == 0
                    u_hx(noCoverageIndex(i,1),counter+1) = vmax;
                    u_hy(noCoverageIndex(i,1),counter+1) = 0;
                else if counter+1-max(find(u_hx(noCoverageIndex(i,1),:)==-returnSpeed))<=70
                        u_hx(noCoverageIndex(i,1),counter+1) = 0;
                        u_hy(noCoverageIndex(i,1),counter+1) = -returnSpeed;
                    else
                        u_hx(noCoverageIndex(i,1),counter+1) = returnSpeed;
                        u_hy(noCoverageIndex(i,1),counter+1) = 0;
                    end
                end
            else if Px_h(noCoverageIndex(i,1),counter)>18000
                    if counter+1-max(find(u_hx(noCoverageIndex(i,1),:)==vmax))<=70
                        u_hx(noCoverageIndex(i,1),counter+1) = 0;
                        u_hy(noCoverageIndex(i,1),counter+1) = returnSpeed;
                    else
                        u_hx(noCoverageIndex(i,1),counter+1) = -returnSpeed;
                        u_hy(noCoverageIndex(i,1),counter+1) = 0;
                    end
                end
            end
        end
    end
    
    Px_h(:,counter+1) = Px_h(:,counter)+u_hx(:,counter+1)*dt;
    Py_h(:,counter+1) = Py_h(:,counter)+u_hy(:,counter+1)*dt;
    Angle_h(:,counter+1) = pi+atan((circley-Py_h(:,counter+1))./(circlex-Px_h(:,counter+1)));
    veangle_h(:,counter+1) = real(atan(u_hy(:,counter+1)./u_hx(:,counter+1)));
    
    for i=1:noCoverageNumber
        if Px_h(noCoverageIndex(i,1),counter)>=1000 & Px_h(noCoverageIndex(i,1),counter)<=18000
            veangle_h(noCoverageIndex(i,1),counter+1) = pi;
        else if Px_h(noCoverageIndex(i,1),counter)<1000
                if firstCoverage(noCoverageIndex(i,1),1) == 0
                    veangle_h(noCoverageIndex(i,1),counter+1) = 0;
                else if counter+1-max(find(u_hx(noCoverageIndex(i,1),:)==-returnSpeed))<=70
                        veangle_h(noCoverageIndex(i,1),counter+1) = -pi/2;
                    else
                        veangle_h(noCoverageIndex(i,1),counter+1) = 0;
                    end
                end
            else if Px_h(noCoverageIndex(i,1),counter)>18000
                    if counter+1-max(find(u_hx(noCoverageIndex(i,1),:)==vmax))<=70
                        veangle_h(noCoverageIndex(i,1),counter+1) = pi/2;
                    else
                        veangle_h(noCoverageIndex(i,1),counter+1) = pi;
                    end
                end
            end
        end
    end
    
    %     %% 判断无人机执行覆盖还是返航任务
    %
    %     for i=1:n*batch
    %
    %         if (Px_h(i,counter)>=1000 & fugai(i)==0)
    %             if u_hy(i,counter)>0 & veangle_h(i,counter)< anglestar
    %                 veangle_h(i,counter+1) = min(veangle_h(i,counter)+pi/90,anglestar);
    %             else if u_hy(i,counter)>0 & veangle_h(i,counter)>= anglestar
    %                     veangle_h(i,counter+1) = anglestar;
    %                 else if u_hy(i,counter)<=0 & veangle_h(i,counter)> angleend-2*pi
    %                         veangle_h(i,counter+1) = veangle_h(i,counter)+pi/30;
    %
    %
    %                     end
    %                 end
    %             end
    %             u_hx(i,counter+1) = 2.5*real(vmax*cos(veangle_h(i,counter+1)));
    %             u_hy(i,counter+1) = 2.5*real(vmax*sin(veangle_h(i,counter+1)));
    %             Px_h(i,counter+1) = Px_h(i,counter)+u_hx(i,counter+1)*dt;
    %             Py_h(i,counter+1) = Py_h(i,counter)+u_hy(i,counter+1)*dt;
    %             Angle_h(i,counter+1) = pi+atan((circley-Py_h(i,counter+1))./(circlex-Px_h(i,counter+1)));
    %             Angle(i) = Angle_h(i,counter+1);
    %
    %         else if Px_h(i,counter)<1000 & veangle_h(i,counter)~= 0 & fugai(i)==0
    %                 if veangle_h(i,counter) > 0
    %                     veangle_h(i,counter+1) = min(veangle_h(i,counter)+pi/180,2*pi);
    %                     veangle_h(i,counter+1) = mod(veangle_h(i,counter+1),2*pi);
    %                     u_hx(i,counter+1) = 2.5*real(vmax*cos(veangle_h(i,counter+1)));
    %                     u_hy(i,counter+1) = 2.5*real(vmax*sin(veangle_h(i,counter+1)));
    %                     Px_h(i,counter+1) = Px_h(i,counter)+u_hx(i,counter+1)*dt;
    %                     Py_h(i,counter+1) = Py_h(i,counter)+u_hy(i,counter+1)*dt;
    %                     Angle_h(i,counter+1) = pi+atan((circley-Py_h(i,counter+1))./(circlex-Px_h(i,counter+1)));
    %                     Angle(i) = Angle_h(i,counter+1);
    %                 else if veangle_h(i,counter) < 0
    %                         veangle_h(i,counter+1) = max(veangle_h(i,counter)-pi/180,-2*pi);
    %                         u_hx(i,counter+1) = real(vmax*cos(-veangle_h(i,counter+1)));
    %                         u_hy(i,counter+1) =real( -vmax*sin(-veangle_h(i,counter+1)));
    %                         Px_h(i,counter+1) = Px_h(i,counter)+u_hx(i,counter+1)*dt;
    %                         Py_h(i,counter+1) = Py_h(i,counter)+u_hy(i,counter+1)*dt;
    %                         Angle_h(i,counter+1) = pi+atan((circley-Py_h(i,counter+1))./(circlex-Px_h(i,counter+1)));
    %                         Angle(i) = Angle_h(i,counter+1);
    %                     end
    %                 end
    %             else if Px_h(i,counter)<1000 & veangle_h(i,counter)== 0 & fugai(i)==0
    %
    %                     veangle_h(i,counter+1) = 0;
    %                     u_hx(i,counter+1) = vmax;
    %                     u_hy(i,counter+1) = 0;
    %                     Px_h(i,counter+1) = Px_h(i,counter)+u_hx(i,counter+1)*dt;
    %                     Py_h(i,counter+1) = Py_h(i,counter)+u_hy(i,counter+1)*dt;
    %                     Angle_h(i,counter+1) = pi+atan((circley-Py_h(i,counter+1))./(circlex-Px_h(i,counter+1)));
    %                     Angle(i) = Angle_h(i,counter+1);
    %                 end
    %             end
    %         end
    %     end
    %% 覆盖率
    chong = 0;
    cyfg = [];
    jd = [];
    bcyfg = find(coverageIndicator==1);
    for j=1:size(bcyfg,1)
        cyfg(j,1) = Angle_h(bcyfg(j),counter+1);
    end
    Anglerate = sort(cyfg);
    zsfu = find(Anglerate>anglestar & Anglerate<angleend);
    for l = 1:size(zsfu,1)
        jd(l,1) =  Anglerate(zsfu(l));
    end
    if size(jd,1)==0
        coverage(counter)=0;
    else
        if jd(1)-anglestar<cov/2
            chong= chong+anglestar-jd(1)+cov/2;
        else chong=chong;
        end
        for i=2:size(jd,1)
            if jd(i)-jd(i-1)<cov
                chong=chong+jd(i-1)+cov-jd(i);
            else chong=chong;
            end
        end
        if angleend-jd(end)<cov/2
            chong=chong+jd(end)+cov/2-angleend;
        else chong=chong;
        end
        coverage(counter)=(cov*size(jd,1)-chong)/(pi/6);
    end
    
    if coverage(counter) >=0.85
        shijian = shijian+1;
        shicov(counter) = shijian/counter;
    else shijian = shijian;
        shicov(counter) = shijian/counter;
    end
    
    for k=1:n
        if Angle_h(k,counter+1)<angleend & Angle_h(k,counter+1)> anglestar & coverageIndicator(k)==1
            xshang(k)=circlex+r*cos(Angle_h(k,counter+1)-cov/2);
            yshang(k)=circley+r*sin(Angle_h(k,counter+1)-cov/2);
            xxia(k)=circlex+r*cos(Angle_h(k,counter+1)+cov/2);
            yxia(k)=circley+r*sin(Angle_h(k,counter+1)+cov/2);
            qwer=[circley,yshang(k),yxia(k),circley];
            qwe=[circlex,xshang(k),xxia(k),circlex];
            set(verCellHandle(k), 'XData',qwe,'YData',qwer);
        else qwer=[];
            qwe=[];
            set(verCellHandle(k), 'XData',qwe,'YData',qwer);
        end
    end
    set(lamdaHandle,'XData',1:counter,'YData',lamde_h)
    set(coverageHandle,'XData',1:counter,'YData',coverage);
    set(time_coverageHandle,'XData',1:counter,'YData',shicov);
    drawnow
end
