function [ux,uy] = covO3(Px,Py,cx,cy,connect)
%区域覆盖控制律，输入为智能体前一时刻的位置、
%速度方向、维诺区域的质心(最优位置)，速度
%输出为无人机的速度方向
ux = cx-Px; %x轴方向速度
uy = cy-Py; %y轴方向速度
distance = sqrt(ux.^2+uy.^2);
number = size(Px,2);
for k = 1:number
    if distance(1,k) >connect
        ux(1,k) = connect*ux(1,k)/distance(1,k);
        uy(1,k) = connect*uy(1,k)/distance(1,k);
    else
    end
end
end