close all;
clear all;
%% 覆盖扇面作图
r = 3000; %雷达半径  %速度167m/s
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
axis([0,21000,0,11000]); %画图范围
hold on;
%% 无人机初始状态设置
n = 20; %每一批次无人机数量
batch = 1; %无人机批次
sd = 6;
rng(sd);
%Px = rand(n,1)*100; %无人机x轴位置
Px = rand(n,1)*900; %无人机x轴位置

%Py = rand(n,1)*2000+4000; %无人机y轴位置
%Py = rand(n,1)*2000+13000; %无人机y轴位置
Py = rand(n,1)*900+5000; %无人机y轴位置
%Py = rand(n,1)*100+5000; %无人机y轴位置
%Py = rand(n,1)*900+1000; %无人机y轴位置
%Py(11:20,1) = rand(10,1)*900+8000;

%无人机开始执行覆盖任务和结束覆盖任务区域
%plot(zeros(1,1001)+1000,0:16000/1000:16000,'--');
%plot(zeros(1,1001)+18000,0:16000/1000:16000,'--');
plot(zeros(1,1001)+1000,0:11000/1000:11000,'--');
plot(zeros(1,1001)+18000,0:11000/1000:11000,'--');
%无人机初始角度
Angle = pi+atan((circley-Py)./(circlex-Px));
%% 作图设置
%无人机覆盖时扇面颜色设置
cellColors = zeros(n*batch,3);
for counter = 1:n
cellColors(counter,:)=[49/255,198/255,159/255];
end
%无人机轨迹设置
%currHandle = plot(Px,Py,'>','linewidth',2);
currHandle = plot(zeros(4,n),zeros(4,n),'LineWidth',1.5,'color','b');

titleHandle = title('无人机轨迹');
%无人机覆盖扇面作图设置
verCellHandle = zeros(n*batch,3);
for counter = 1:n*batch 
verCellHandle(counter)=patch(zeros(n*batch,1),zeros(n*batch,1),cellColors(counter,:));
end

%% 参数设置
% 连通保持控制器参数设定
R = 500;%通信半径
%R = 2000;%通信半径
deta = 0.1;%通信边界处的边权大小，越小效果越好
Epsilon = 0.1;%最小代数连通度
vmax = 27.8;%连通保持最大速度（用于限幅）
veangle = zeros(n,1);
totalTime = 1200;%仿真总时长
%dt = 0.5;%控制器更新频率
dt = 1;%控制器更新频率
numIterations = ceil(totalTime/dt)+1;%迭代次数
t = 0;
%无人机位置、角度数据储存
Px_h = zeros(n*batch,numIterations);
Py_h = zeros(n*batch,numIterations);
Angle_h = zeros(n*batch,numIterations);
Px_h(:,1) = Px;
Py_h(:,1) = Py;
Angle_h(:,1) = Angle;
%日志向量定义（记录控制量）
ue_hx = zeros(n*batch,numIterations);
ue_hy = zeros(n*batch,numIterations);
ue_hx(1:n,1) = zeros(n*batch,1)+vmax;
ue_hy(1:n,1) = zeros(n*batch,1);
uc_hx = zeros(n*batch,numIterations);
uc_hy = zeros(n*batch,numIterations);
u_hx = zeros(n*batch,numIterations);
u_hy = zeros(n*batch,numIterations);
u_hx(1:n,1) = ue_hx(1:n,1)+uc_hx(1:n,1);
u_hy(1:n,1) = ue_hy(1:n,1)+uc_hy(1:n,1);
veangle_h = zeros(n*batch,numIterations);
%初始损失和连通度计算
x = [Px(1:n) Py(1:n)];
[L,A,d] = LaplaMat(x,R,deta);
[vector,value] = eig(L);
lamde_h = zeros(numIterations,1);
lamde_h(1) = value(2,2);
disp([num2str(t),'时刻的连通度为',num2str(value(2,2))]);
number = 0; %算法执行次数
fugai = zeros(n,1)+1; %判断无人机是否参与覆盖，参与赋值1，不参与赋值0
shijian = 0;
for counter=1:numIterations
if max(Px_h(:,counter))>18000
    break;
else
    fugai = zeros(n,1)+1; %判断无人机是否参与覆盖，参与赋值1，不参与赋值0
 %   set(currHandle,'XData',Px_h(1:n,counter),'YData',Py_h(1:n,counter));
%    edges = [-0.15,0.32,-0.15,-0.15;-0.15,0,0.15,-0.15]; 
%    edges = [-150,320,-150,-150;-150,0,150,-150]/2;
% 无人机位置
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
    end       
%% 角度覆盖控制律
   ue = ccangle(Px_h(1:n,counter),Py_h(1:n,counter),Angle_h(1:n,counter),ue_hy(1:n,counter),veangle_h(1:n,counter),anglestar,angleend,R,circlex,circley,vmax,cov);
   ue_hx(1:n,counter+1)=ue(:,1);
   ue_hy(1:n,counter+1)=ue(:,2);
%% 判断无人机控制率是否改变，使无人机轨迹平滑
for i=1:n
if abs(ue_hx(i,counter+1)-ue_hx(i,counter))<0.01
    ue_hx(i,counter+1) = ue_hx(i,counter);
end
if abs(ue_hy(i,counter+1)-ue_hy(i,counter))<0.01
    ue_hy(i,counter+1) = ue_hy(i,counter);
end
end

%% 分段连通约束控制
lamde2est=ones(n,1)*value(2,2);
v2=vector(:,2);
uc=connect_preserve(lamde2est,v2,x,d,A,R,deta,Epsilon);
%限幅处理
for agent=1:n
    if norm(uc(agent,:))>vmax
        uc(agent,:)=vmax*uc(agent,:)./norm(uc(agent,:));
    end
end
uc_hx(1:n,counter+1)=uc(:,1);
uc_hy(1:n,counter+1)=uc(:,2);

%% 总控制
u = 3*uc+ue;%控制律叠加
% u = ue;
%u = ue;%控制律叠加
u_hx(1:n,counter+1) = u(:,1);
u_hy(1:n,counter+1) = u(:,2);
Px_h(1:n,counter+1) = Px_h(1:n,counter)+u(:,1)*dt;
Py_h(1:n,counter+1) = Py_h(1:n,counter)+u(:,2)*dt;
Angle_h(:,counter+1) = pi+atan((circley-Py_h(:,counter+1))./(circlex-Px_h(:,counter+1)));
Angle = Angle_h(1:n,counter+1);
% if sum(u_hy(1:n,counter+1)./(zeros(n,1)+vmax) > 1) 
%     disp("out of range")
% end
veangle_h(1:n,counter+1) = asin(u_hy(1:n,counter+1)./(zeros(n,1)+vmax));  

%% 判断无人机是否执行覆盖任务
for i=1:n
    if Px_h(i,counter)<=1000
        fugai(i,1) = 0;
        u_hx(i,counter+1) = u_hx(i,counter);
        u_hy(i,counter+1) = u_hy(i,counter);
        Px_h(i,counter+1) = Px_h(i,counter)+u_hx(i,counter+1)*dt;
        Py_h(i,counter+1) = Py_h(i,counter)+u_hy(i,counter+1)*dt;
        Angle_h(i,counter+1) = pi+atan((circley-Py_h(i,counter+1))./(circlex-Px_h(i,counter+1)));
        Angle(i) = Angle_h(i,counter+1);
        veangle_h(i,counter+1) = asin(u_hy(i,counter+1)/vmax);
    end
end

%% 计算下一时刻的连通度
[L,A,d]=LaplaMat([Px_h(1:n,counter+1) Py_h(1:n,counter+1)],R,deta);
[vector,value]=eig(L);
disp([num2str((counter+1)),'时刻的连通度为',num2str(value(2,2))])
lamde_h(counter+1)=value(2,2);
%计算下一时刻的覆盖损失
%位置与时间更新
x=[Px_h(1:n,counter+1) Py_h(1:n,counter+1)]; 
%% 覆盖率
chong = 0; % 角度重叠
cyfg = [];
jd = [];
bcyfg = find(fugai==1);
for j=1:size(bcyfg,1)
    cyfg(j,1) = Angle(bcyfg(j));
end

Anglerate = sort(cyfg);
% for j = 1:size(Anglerate, 1)
%     disp(Anglerate(j, 1))
% end

zsfu = find(Anglerate>anglestar & Anglerate<angleend);
for l = 1:size(zsfu,1)
    jd(l,1) =  Anglerate(zsfu(l));
end
if size(jd,1)==0
    coverage(counter)=0;
else
    if jd(1)-anglestar<cov/2
        chong= chong+anglestar-jd(1)+cov/2;
    end
    for i=2:size(jd,1)
        if jd(i)-jd(i-1)<cov
            chong=chong+jd(i-1)+cov-jd(i);
        end
    end
    if angleend-jd(end)<cov/2
        chong=chong+jd(end)+cov/2-angleend;
    end
    coverage(counter)=(cov*size(jd,1)-chong)/(pi/6);
end
if coverage(counter) >=0.85
    shijian = shijian+1;
    shicov(counter) = shijian/counter;
end
for k=1:n
    if Angle(i)<angleend & Angle(i)> anglestar
        xshang(k)=circlex+r*cos(Angle(k,1)-cov/2);
        yshang(k)=circley+r*sin(Angle(k,1)-cov/2);
        xxia(k)=circlex+r*cos(Angle(k,1)+cov/2);
        yxia(k)=circley+r*sin(Angle(k,1)+cov/2);
        qwer=[circley,yshang(k),yxia(k),circley];
        qwe=[circlex,xshang(k),xxia(k),circlex];
        set(verCellHandle(k), 'XData',qwe,'YData',qwer);
    else
    end
end
drawnow
end
number=number+1;
end

wei=size(coverage,2);
coverage(1,wei)
shicov(1,wei)
figure(2);
plot(0:1:wei-1,lamde_h(1:wei),'LineWidth',1.5,'color','r');
hold on;
plot(0:1:wei-1,zeros(1,wei)+Epsilon,'--','LineWidth',1.5,'color','b');
title('代数连通度变化');
xlabel('迭代次数');
ylabel('\lambda_2');
xlim([0,number])
ylim([0,max(lamde_h)+0.1])

figure(3);
plot(0:1:wei-1,coverage,'-','LineWidth',1.5,'color','r');
hold on;
plot(0:1:wei-1,zeros(1,wei)+0.8,'--','LineWidth',1.5,'color','b');
axis([0,number,0,1.1]);
title('覆盖率变化');
xlabel('迭代次数');
ylabel('覆盖率');

figure(4);
plot(0:1:wei-1,shicov,'-','LineWidth',1.5,'color','r');
hold on;
plot(0:1:wei-1,zeros(1,wei)+0.85,'--','LineWidth',1.5,'color','b');
title('时间覆盖率');