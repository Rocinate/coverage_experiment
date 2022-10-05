function [speedAngele,optimal] = covO2(Px,Py,Pa,cx,cy)
%区域覆盖控制律，输入为智能体前一时刻的位置、
%速度方向、维诺区域的质心(最优位置)，速度
%输出为无人机的速度方向
v_x = cx-Px; %x轴方向速度
v_y = cy-Py; %y轴方向速度
number = size(cx,2);
speedAngele = zeros(1,number); %最优的速度方向
optimal = zeros(1,number);
%计算智能体的速度方向
for k = 1:number        
    if v_x(1,k) == 0 && v_y(1,k) == 0
        speedAngele(1,k) = 0; %若位置不变 维持原运动方向
        optimal(1,k) = 1;
    elseif v_x(1,k) ==0 && v_y(1,k)>0
        speedAngele(1,k) = pi/2;
    elseif v_x(1,k) ==0 && v_y(1,k)<0
        speedAngele(1,k) = 3*pi/2;
    elseif v_x(1,k)>0 && v_y(1,k)>=0
        speedAngele(1,k) = atan(v_y(1,k)/v_x(1,k));
    elseif v_x(1,k)<0 && v_y(1,k)>=0
        %speedAngele(1,k) = atan(v_y(1,k)/v_x(1,k))-pi;
        speedAngele(1,k) = pi-atan(abs(v_y(1,k)/v_x(1,k)));
    elseif v_x(1,k)<0 && v_y(1,k)<=0
        %speedAngele(1,k) = atan(v_y(1,k)/v_x(1,k))+pi;
        speedAngele(1,k) = atan(abs(v_y(1,k)/v_x(1,k)))+pi;
    elseif v_x(1,k)<0 && v_y(1,k)>=0
       % speedAngele(1,k) = atan(v_y(1,k)/v_x(1,k))+2*pi;
       speedAngele(1,k) = 2*pi-atan(abs(v_y(1,k)/v_x(1,k)));
    end
end
%智能体角度变化上界
upper_bound = 2*pi;
%upper_bound = pi/5;
speed_change = speedAngele-Pa;
for k=1:number
    if speed_change(1,k) >upper_bound && speed_change(1,k)<=pi
        speedAngele(1,k) = Pa(1,k)+upper_bound;
    elseif speed_change(1,k) >upper_bound && speed_change(1,k)>pi
        speedAngele(1,k) = Pa(1,k)-upper_bound;
    elseif speed_change(1,k) <-upper_bound && speed_change(1,k)<=pi
        speedAngele(1,k) = Pa(1,k)-upper_bound;
    elseif speed_change(1,k) <-upper_bound && speed_change(1,k)>pi
        speedAngele(1,k) = Pa(1,k)+upper_bound;
    end
    speedAngele(1,k) = mod(speedAngele(1,k),2*pi);
end
end