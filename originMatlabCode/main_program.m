close all;
clear all;
%% 参数设置
numIterations = 50;  %迭代次数
xrange = 0.75;  yrange = 1;  %覆盖范围-xrange~xrange， -yrange~yrange
crs = [ -xrange, -yrange; -xrange, yrange; xrange, yrange; xrange, -yrange];  %覆盖范围顶点坐标
n = 4; %无人机数量  
Px = zeros(4,1)-0.3;  Py = [0.45; 0.15; -0.15; -0.5];  Pz = zeros(n,1);  %无人机初始状态(位置+角度)
%Px = zeros(n,1)-0.3;  Py = [0.5; 0.4; 0.3; 0.2; 0.1];  Pz = zeros(n,1);  %无人机初始状态(位置+角度)
Pq = zeros(n,1);  %上次迭代的最后控制率
vv = [0.05; 0.05; 0.05; 0.05];  %无人机速度
%vv = [0.05; 0.05; 0.05; 0.05; 0.05; 0.05];  %无人机速度
ww = 0.5;  %半径参数设置，轨道半径 vv/ww
Pathx = [];  Pathy = [];  Pathz = [];  %整体轨迹储存
volume = 0.05;  %机器人体积半径
%coveraR = 0.65;  %项目覆盖率指标
%% 作图设置
verCellHandle = zeros(n,1);  %维诺划分作图句柄设置
cellColors = cool(n); %维诺划分有颜色
%cellColors = zeros(5,3)+1;  %维诺划分无颜色    
for i = 1:n
    verCellHandle(i)  = patch(Px(i),Py(i),cellColors(i,:));
    hold on
end
pthHandle = zeros(n,1);  %轨迹作图句柄设置  
 for i = 1:n
     pathHandle(i)  = plot(Px(i),Py(i),'-','color',cellColors(i,:)*.8,'linewidth',1.5);
 end
goalHandle = plot(Px,Py,'+','linewidth',2);  %无人机初始位置作图
currHandle = plot(Px(1),Py(1),'-','linewidth',2);  %无人机轨迹作图
titleHandle = title('trajectory');
%% 算法迭代
figure(1);
for counter = 1:numIterations
    [v,c] = VoronoiBounded(Px,Py,crs);  %真实位置的voronoi划分计算，第c个无人机维诺划分顶点坐标v
    for i = 1:n  %虚拟位置计算
        Pendx(i,1) = Px(i)-(vv(i)/ww)*(sin(Pz(i)));
        Pendy(i,1) = Py(i)+(vv(i)/ww)*(cos(Pz(i)));
        Pendz(i,1) = Pz(i);
    end
    [v2,c2] = VoronoiBounded(Pendx,Pendy,crs);  %虚拟中心voronoi划分计算
    %cxvbx = zeros(n,1);  cxvby = zeros(n,1);  %目标位置
    for i = 1:n
        [cx2,cy2] = PolyCentroid(v2(c2{i},1),v2(c2{i},2));  %虚拟维诺划分中心
        %cxvbx(i) = cx2;  cxvby(i) = cy2;
        %Centroidx(counter,i) = cx2;  Centroidy(counter,i) = cy2;
    end
    %set(goalHandle,'XData',cxvbx,'YData',cxvby);  %plot goal position
    for i = 1:n  %真实位置的voronoi划分作图
        %set(verCellHandle(i), 'XData',v2(c2{i},1),'YData',v2(c2{i},2));  %虚拟位置的voronoi划分作图
        set(verCellHandle(i), 'XData',v(c{i},1),'YData',v(c{i},2));  %真实位置的voronoi划分作图
    end  
    for i = 1:n %计算无人机控制率和位置
        [cx,cy] = PolyCentroid(v(c{i},1),v(c{i},2)); %实际维诺划分计算质心
        [P1,P2,P3,P4] = casadiOCP(Px,Py,Pz,i,v,c,Pq(i),cx,cy,vv,ww,v2,c2,volume); %计算下一步控制率
        Pq(i) = P4;  %上次迭代的最后控制率
        %Pqwe(counter,i) = P4;  %上次迭代的最后控制率全部储存
        zt = size(P1,1);
        for kt=1:zt
            Ppz(i) = P3(kt);  %实际角度  
            Pathz(zt*(counter-1)+kt,i) = Ppz(i);   
            Ppx(i) = P1(kt)+(vv(i)/ww)*(sin(P3(kt)));  %实际x轴位置
            Pathx(zt*(counter-1)+kt,i) = Ppx(i);  %位置储存
            Ppy(i) = P2(kt)-(vv(i)/ww)*(cos(P3(kt)));   
            Pathy(zt*(counter-1)+kt,i) = Ppy(i);
            set(currHandle,'XData',Ppx(i),'YData',Ppy(i));  %画图无人机轨迹
            %covpx(i,(counter-1)*zt+kt)=Ppx(i);  
            %covpy(i,(counter-1)*zt+kt)=Ppy(i);  %%覆盖率计算
            xD = [get(pathHandle(i),'XData'),Ppx(i)];
            yD = [get(pathHandle(i),'YData'),Ppy(i)];
            set(pathHandle(i),'XData',xD,'YData',yD);  %plot path position
         end
    end
    for i = 1:n  %无人机每次迭代最后时刻实际位置
        Px(i) = Ppx(i);  Py(i) = Ppy(i);  Pz(i) = Ppz(i);
    end
    axis equal
    axis([-xrange,xrange,-yrange,yrange]);
    drawnow
end
%% 假设每个智能体的覆盖空间为一个圆，计算其覆盖率
% sj_point = rand(2000,2);
% xs = 1.5*sj_point(:,1)-0.75;
% y = 2*sj_point(:,2)-1;
% for j = 1:size(covpx,2) 
%     m = 0;
%     for i = 1:size(sj_point,1)
%         dis = (covpx(:,j)-sj_point(i,1)*(zeros(n,1)+1)).^2+(covpy(:,j)-sj_point(i,2)*(zeros(n,1)+1)).^2;
%         if dis(1) < coveraR^2 | dis(2) < coveraR^2 | dis(3) < coveraR^2 | dis(4) < coveraR^2
%             m = m+1;
%         end
%     end
%     coverage(j) = m/2000;
% end
% figure (2)
% plot(1:1:size(covpx,2),coverage,'-');
% hold on;
% plot(1:1:size(covpx,2),zeros(size(covpx,2),1)+0.85,'--')
%% 损失函数计算
figure(3)
[jk,nm] = meshgrid(-xrange:2*xrange/10:xrange,-yrange:2*yrange/10:yrange);
jk = jk(:);  nm = nm(:); 
for j = 1:size(Pathx,1)
    for k = 1:size(jk,1)
        cost(k) = min(sqrt((zeros(n,1)+jk(k)-Pathx(j,:)').^2+(zeros(n,1)+nm(k)-Pathy(j,:)').^2));           
    end
    costfunction(j) = sum(cost);
end
plot(1:1:size(costfunction,2),costfunction,'-');
%% 机器人离维诺边界的最小距离
figure(4)
for j = 1:size(Pathx,1)
    if mod(j+6,7) == 0  %由casadi的N确定，除数为N+1
        [vrea,crea] = VoronoiBounded(Pathx(j,:)',Pathy(j,:)',crs);  %真实位置的voronoi划分计算，第c个无人机维诺划分顶点坐标v
    end
    for k = 1:n
        voronoix = vrea(crea{k},1);  %维诺边界x端点坐标，首尾重复 
        voronoiy = vrea(crea{k},2);  %维诺边界y端点坐标，
        for p = 1:(size(voronoix,1)-1)
            safedis=[];
            for m = 1:(size(voronoix,1)-1)
                a = voronoiy(m+1)-voronoiy(m);
                b = voronoix(m+1)-voronoix(m);
                % 机器人位置距离各维诺边界的距离
                safedis(m) = abs((Pathy(j,k)-voronoiy(m))*b-((Pathx(j,k)-voronoix(m))*a))/sqrt(a^2+b^2);
                safe(j,k) = min(safedis);
            end
        end
    end
end
for j = 1:n
    plot(1:1:size(Pathx,1),safe(:,j),'-');
    hold on;
end
