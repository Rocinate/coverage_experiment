function [newCx,newCy] = ConnectConstraint(Px,Py,cx,cy,connect)
%计算无人机实际位置对应的虚拟位置
%输入：无人机虚拟位置Px,Py；无人机无约束的最优位置cx,cy；移动距离约束;connect
%输出：约束后的无人机位置：newCx,newCy
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
end
