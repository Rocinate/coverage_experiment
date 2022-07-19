function ue= newAngleCoverage(Px,Py,Angle,ue_hy,veangle,anglestar,angleend,circlex,circley,vmax,cov,R)
%输出:ue：角度覆盖控制y轴方向控制量，为n行1列的向量
%输入:Px和Py分别表示个体的x。
n = size(Px,1);   
ue = zeros(n,1);   %!!!!!!!!!!修改
opti = zeros(n,1);%无约束下无人机最优角度
Rmax = 1*R; %角度覆盖无人机的通信范围
for i = 1:n
    %% 判断无人机是否处于覆盖扇面之外，采取不同的策略得到最优覆盖角度
    %%%覆盖扇面外的无人机最优角度
    if Angle(i) < anglestar 
        opti(i) = anglestar+cov/2;
    elseif Angle(i) > angleend
            opti(i) = angleend-cov/2;
    else
    %%%覆盖扇面内的无人机最优角度
    xpos = [];
    ypos = [];
    ang = [];
    d = sqrt((zeros(n,1)+Px(i)-Px).^2+(zeros(n,1)+Py(i)-Py).^2); %无人机之间的距离
    ind = find(d<=Rmax);%邻居集索引（包括自己）
    q = size(ind,1);
    if q == 1
        ang(q,1) = anglestar;
        ang(q+1,1) = angleend;
        ang(q+2,1) = Angle(i);
        ang = sort(ang);
        w = find(ang==Angle(i));
        opti(i) = (ang(w-1)+ang(w+1))/2;
    else
        ind(ind==i) = [];%删去自己
        for j = 1:q-1
            agent = ind(j);%邻居索引,边（i，agent）
            xpos(j,1) = Px(agent);
            ypos(j,1) = Py(agent);
            ang(j,1) = Angle(agent);
        end
        ang(q,1) = anglestar;
        ang(q+1,1) = angleend;
        %测试控制率，边界比较平滑
        %ang(q+1,1)=max(pi+atan((circley-Py(i)-Rmax)/(circlex-Px(i))),anglestar);
        %ang(q+2,1)=min(pi+atan((circley-Py(i)+Rmax)/(circlex-Px(i))),angleend);
        ang(q+2,1) = Angle(i);
        ang = sort(ang);
        w = find(ang==Angle(i));
        opti(i) = (ang(w-1)+ang(w+1))/2;
    end
    end
end
%% 影响程度受距离的影响，距离雷达越近，照常的影响越小
beta = 10000; %距离雷达最远时，y方向的速度增益   %!!!!!!!!!!修改
gamma = 0; %距离雷达最近时，y方向的速度增益   %!!!!!!!!!!修改
positionStar = 1000; %无人机开始执行覆盖的x轴初始位置   %!!!!!!!!!!修改
positonEnd = 18000; %无人机结束执行覆盖的x轴结束位置   %!!!!!!!!!!修改
gradient = (gamma-beta)/(positonEnd-positionStar);  %!!!!!!!!!!修改
intercept = beta-positionStar*gradient;  %!!!!!!!!!!修改
for k=1:n
    alphe = gradient*Px(k)+intercept;  %!!!!!!!!!!修改
    if ue_hy(k)*(Angle(k)-opti(k))>=0 %相同运动趋势
        ue(k) = ue_hy(k)+alphe*(Angle(k)-opti(k));
        ue(k) = sign(ue(k))*min(abs(ue(k)),vmax);
    else
        ue(k) = -ue_hy(k)+alphe*(Angle(k)-opti(k)); %相反运动趋势
        ue(k) = sign(ue(k))*min(abs(ue(k)),vmax);
    end
end
%% 保证无人机相邻时刻转角的幅度在pi/12之内
for k=1:n
    ank = atan(ue(k)/vmax);
    if abs(ank-veangle(k))<=pi/30  %每次速度转变的角度上界
        ue(k) = ue(k);
    else
        newangle = veangle(k)+sign(Angle(k)-opti(k))*pi/30;
        ue(k) = vmax*tan(newangle);
    end
end  
%% 保证无人机速度在-pi/3-pi/3
for i = 1:n
    anq = atan(ue(i)/vmax);
    if anq < -pi/3
        ue(i) = vmax*tan(-pi/3);
    elseif anq > pi/3
        ue(i) = vmax*tan(pi/3);
    else
        ue(i) = ue(i);
    end        
end
