function [P1,P2,P3,P4] = casadiOCP(Px,Py,Pz,i,v,c,PQW,cx,cy,vv,ww,v2,c2,volume)
import casadi.*
%% 参数定义
T = 3; % Time horizon
N = 6; % number of control intervals
x1 = SX.sym('x1');
x2 = SX.sym('x2');
x3 = SX.sym('x3');
x = [x1; x2; x3];
u = SX.sym('u');
%% Model equations
xdot = [(1-u/ww)*vv(i)*cos(x3);(1-u/ww)*vv(i)*sin(x3);u]; %动力学模型
%% Objective term
xmin = min(v2(c2{i},1));  xmax = max (v2(c2{i},1));
ymin = min(v2(c2{i},2));  ymax = max(v2(c2{i},2));
[Qx,Qy] = meshgrid(xmin:(xmax-xmin)/14:xmax,ymin:(ymax-ymin)/9:ymax);
Qx = Qx(:);  Qy = Qy(:);  QZZ = [Qx,Qy];  KK = 1;  QQ = zeros(1,2);
for qq = 1:size(Qx,1)
    if inpolygon(QZZ(qq,1),QZZ(qq,2),v2(c2{i},1),v2(c2{i},2))
        QQ(KK,:) = QZZ(qq,:);
        KK = KK+1;
    end
end
L = 0;
for kq = 1:size(QQ,1)
    L = L+sqrt((x1-QQ(kq,1))^2+(x2-QQ(kq,2))^2);
end
%L = L+u^2;
%L = sqrt((x1-cx)^2+(x2-cy)^2);%损失函数
%L = L+u^2;
%% Formulate discrete time dynamics
if false
   % CVODES from the SUNDIALS suite
   dae = struct('x',x,'p',u,'ode',xdot,'quad',L);
   opts = struct('tf',T/N);
   F = integrator('F', 'cvodes', dae, opts);
else
   % Fixed step Runge-Kutta 4 integrator
   M = 4; % RK4 steps per interval
   DT = T/N/M;
   f = Function('f', {x, u}, {xdot, L});
   X0 = MX.sym('X0', 3);
   U = MX.sym('U');
   X = X0;
   Q = 0;
   for j=1:M
       [k1, k1_q] = f(X, U);
       [k2, k2_q] = f(X + DT/2 * k1, U);
       [k3, k3_q] = f(X + DT/2 * k2, U);
       [k4, k4_q] = f(X + DT * k3, U);
       X = X+DT/6*(k1 +2*k2 +2*k3 +k4);
       Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
    end
    F = Function('F', {X0, U}, {X, Q}, {'x0','p'}, {'xf', 'qf'});
end
%% Start with an empty NLP
w = {};
w0 = [];
lbw = [];
ubw = [];
J = 0;
g = {};
lbg = [];
ubg = []; 
%% Formulate the NLP
Zcx = Px(i)-(vv(i)/ww)*(sin(Pz(i)));  %转为虚拟位置
Zcy = Py(i)+(vv(i)/ww)*(cos(Pz(i)));
Zcz = Pz(i);
Xk= [Zcx; Zcy; Zcz];  %初始状态
voronoix = v(c{i},1);  %维诺边界x端点坐标，首尾重复 
voronoiy = v(c{i},2);  %维诺边界y端点坐标， 
zxc = size(voronoix,1)-1;  %约束条件个数
zzxc = size(voronoix,1)+1;  %n之后增加1和2，便于循环
duanx = zeros(zzxc,1);  %维诺顶点坐标
duany = zeros(zzxc,1);  
for iq = 1:zzxc-1
    duanx(iq) = voronoix(iq);
    duany(iq) = voronoiy(iq);
end
duanx(zzxc) = voronoix(2);
duany(zzxc) = voronoiy(2);
lowbound = zeros(zzxc-2,1);  %约束条件边界
upbound = zeros(zzxc-2,1);
for iqw = 1:(zzxc-2) %避撞约束判断   
     if ((duanx(iqw+2)-duanx(iqw))*(duany(iqw+1)-duany(iqw))-(duany(iqw+2)-duany(iqw))*(duanx(iqw+1)-duanx(iqw))) > 0
         upbound(iqw) = inf;
     else lowbound(iqw) = -inf;
     end
end
for k = 0:N-1
    %% New NLP variable for the control
    Uk = MX.sym(['U_' num2str(k)]);
    w = {w{:}, Uk};
    lbw = [lbw; -0.5];  %控制u上界
    ubw = [ubw;  0.5];  %控制u下界
    w0 = [w0;  PQW];  %控制率初值
    % Integrate till the end of the interval
    Fk = F('x0', Xk, 'p', Uk);
    Xk = Fk.xf;
    J = J+Fk.qf;
    for kZ=1:zxc
        %避撞约束
        g = [g; {[(Xk(1)+(vv(i)/ww)*sin(Xk(3))-duanx(kZ))*(duany(kZ+1)-duany(kZ))-(Xk(2)-(vv(i)/ww)*cos(Xk(3))-duany(kZ))*(duanx(kZ+1)-duanx(kZ))]}];
    end
    for kZ=1:zxc
        %加入无人机自身体积约束的避撞
        g = [g; {[abs((Xk(1)+(vv(i)/ww)*sin(Xk(3))-duanx(kZ))*(duany(kZ+1)-duany(kZ))-(Xk(2)-(vv(i)/ww)*cos(Xk(3))-duany(kZ))*(duanx(kZ+1)-duanx(kZ)))/sqrt((duany(kZ+1)-duany(kZ))^2+(duanx(kZ+1)-duanx(kZ))^2)]}];
    end
    lbg = [lbg; lowbound];
    ubg = [ubg; upbound];      
    for kZ = 1:zxc
        lbg = [lbg; volume];
        ubg = [ubg; inf]; 
    end
end
%% Create an NLP solver
prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
solver = nlpsol('solver', 'ipopt', prob);
%% Solve the NLP
sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,...
             'lbg', lbg, 'ubg', ubg);
w_opt = full(sol.x);
%% 控制率、状态输出
u_opt = w_opt;
x_opt = [Zcx;Zcy;Zcz];
for k=0:N-1
    Fk = F('x0', x_opt(:,end), 'p', u_opt(k+1));
    x_opt = [x_opt, full(Fk.xf)];
end
x1_opt = x_opt(1,:);
x2_opt = x_opt(2,:);
x3_opt =x_opt(3,:);
P1 = x1_opt;  P1 = P1';
P2 = x2_opt;  P2 = P2';
P3 = x3_opt;  P3 = P3';
P4 = u_opt(end);