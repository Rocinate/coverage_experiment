function [covControlX,covControlY] = covFirstOrder(Px,Py,cx,cy,connect)
%区域覆盖控制律，输入为智能体前一时刻的位置、
%速度方向、维诺区域的质心(最优位置)
%输出为无人机的速度方向
distance = sqrt((Px-cx).^2+(Py-cy).^2); %计算虚拟中心和最优位置的距离
uavID = find(distance>connect); %找出不符合约束的无人机编号
num = size(uavID,2);
for k = 1:num %对最优位置进行约束
    NEW = [Px(1,k),Py(1,k)]+([cx(1,uavID(k)),cy(1,uavID(k))]-[Px(1,k),Py(1,k)])...
        /norm([cx(1,uavID(k)),cy(1,uavID(k))]-[Px(1,k),Py(1,k)])*min(connect,distance(uavID(k)));
%     angle = tanh((cx(1,uavID(k))-Px(1,k))/(cy(1,uavID(k))-Py(1,k)));
%     cx(1,uavID(k)) = Px(1,k)+ sign((cx(1,uavID(k))-Px(1,k)))*connect*abs(sin(angle));
%     cy(1,uavID(k)) = Py(1,k)+ sign((cy(1,uavID(k))-Py(1,k)))*connect*abs(cos(angle));
    cx(1,uavID(k)) = NEW(1,1);
    cy(1,uavID(k)) = NEW(1,2);  
end
newCx = cx;
newCy = cy;
covControlX = newCx-Px; %x轴方向速度
covControlY = newCy-Py; %y轴方向速度






distance = sqrt((Px-cx).^2+(Py-cy).^2); %计算虚拟中心和最优位置的距离
uavID = find(distance>connect); %找出不符合约束的无人机编号
num = size(uavID,2);
for k = 1:num %对最优位置进行约束
    NEW = [Px(1,k),Py(1,k)]+([cx(1,uavID(k)),cy(1,uavID(k))]-[Px(1,k),Py(1,k)])...
        /norm([cx(1,uavID(k)),cy(1,uavID(k))]-[Px(1,k),Py(1,k)])*min(connect,distance(uavID(k)));
%     angle = tanh((cx(1,uavID(k))-Px(1,k))/(cy(1,uavID(k))-Py(1,k)));
%     cx(1,uavID(k)) = Px(1,k)+ sign((cx(1,uavID(k))-Px(1,k)))*connect*abs(sin(angle));
%     cy(1,uavID(k)) = Py(1,k)+ sign((cy(1,uavID(k))-Py(1,k)))*connect*abs(cos(angle));
    cx(1,uavID(k)) = NEW(1,1);
    cy(1,uavID(k)) = NEW(1,2);  
end
newCx = cx;
newCy = cy;
covControlX = newCx-Px; %x轴方向速度
covControlY = newCy-Py; %y轴方向速度



end