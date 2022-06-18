function ue= ccangle(Px,Py,Angle,ue_hy,veangle,anglestar,angleend,R,circlex,circley,vmax,cov)
%输出:ue角度覆盖控制控制量，为n行2列的向量，列分别对应x和y
%输入:Px和Py分别表示个体的x，y分量，第一列为x，第二列为y。
n = size(Px,1);
ue = zeros(n,2);
opti = zeros(n,1);%无约束下无人机最优角度
for i = 1:n
    %% 判断无人机是否处于覆盖扇面之外，采取不同的策略得到最优覆盖角度
    %%%覆盖扇面外的无人机最优角度
    if Angle(i) < anglestar 
        opti(i) = anglestar+cov/2;
    elseif Angle(i) > angleend
            opti(i) = angleend-cov/2;
    else
    %%%覆盖扇面内的无人机最优角度
    ang = [];
    d = sqrt((zeros(n,1)+Px(i)-Px).^2+(zeros(n,1)+Py(i)-Py).^2); %无人机之间的距离
    ind = find(d<=R);%邻居集索引（包括自己）
    q = size(ind,1);
    for j = 1:q
        agent = ind(j);%邻居索引,边（i，agent）
        ang(j,1) = Angle(agent);
    end
    ang(q+1,1) = anglestar;
    ang(q+2,1) = angleend;
    ang = sort(ang);
    w = find(ang==Angle(i));
    opti(i) = (ang(w-1)+ang(w+1))/2;
    end
end
beta = 1000; %影响程度受距离的影响，距离雷达越近，照常的影响越小
for k=1:n
    %alphe = (-beta*Px(k)+1000*beta)/20000+beta
    alphe = (0.05-beta)*Px(k)/17100+(beta*181-0.5)/171;
    if ue_hy(k)*(Angle(k)-opti(k))>=0 %相同运动趋势
        ue(k,2) = ue_hy(k)+alphe*(Angle(k)-opti(k));
        ue(k,2) = sign(ue(k,2))*min(abs(ue(k,2)),vmax);
    else
        ue(k,2) = -ue_hy(k)+alphe*(Angle(k)-opti(k)); %相反运动趋势
        ue(k,2) = sign(ue(k,2))*min(abs(ue(k,2)),vmax);
    end
end
%% 保证无人机相邻时刻转角的幅度在pi/6之内
for k=1:n
    ank = asin(ue(k,2)/vmax);
    if abs(ank-veangle(k))<=pi/18 %每次速度转变的角度上界
        ue(k,1) = sqrt(vmax^2-ue(k,2)^2);
    else
        newangle = veangle(k)+sign(Angle(k)-opti(k))*pi/18;
        ue(k,1) = vmax*cos(newangle);
        ue(k,2) = vmax*sin(newangle);
    end
end  
%% 保证无人机速度范围在-pi/3-pi/3
for i = 1:n
    anq = asin(ue(i,2)/vmax);
    if anq < -pi/3
        ue(i,1) = vmax*cos(-pi/3);
        ue(i,2) = vmax*sin(-pi/3);
    elseif anq > pi/3
        ue(i,1) = vmax*cos(pi/3);
        ue(i,2) = vmax*sin(pi/3);
    end        
end
