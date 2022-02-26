function x_opt = Minecasadi(verticesX, verticesY, PositionX, PositionY, Pose, T, N, vv, ww)
verticesX = verticesX';
verticesY = verticesY';
import casadi.*

% Declare model variables
x1 = SX.sym('x1');
x2 = SX.sym('x2');
x3 = SX.sym('x3');
x = [x1; x2; x3];
u = SX.sym('u');

voronoiX=[verticesX(:); verticesX(2)];
voronoiY=[verticesY(:); verticesY(2)];
verticesNum=size(voronoiX,1)-2;

lowbound=zeros(verticesNum,1);
upbound=zeros(verticesNum,1);
for i=1:(verticesNum)
     if ((voronoiX(i+2)-voronoiX(i))*(voronoiY(i+1)-voronoiY(i))-(voronoiY(i+2)-voronoiY(i))*(voronoiX(i+1)-voronoiX(i)))>0
         upbound(i)=inf;
     else lowbound(i)=-inf;
     end
end

% Model equations
xdot = [(1-u/ww)*vv*cos(x3);(1-u/ww)*vv*sin(x3);u];
 
% Objective term

[cx,cy] = PolyCentroid(voronoiX(1:end-1), voronoiY(1:end-1));
L = sqrt((x1 - cx) ^ 2 + (x2 - cy) ^ 2);

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
    X=X+DT/6*(k1 +2*k2 +2*k3 +k4);
    Q = Q + DT/6*(k1_q + 2*k2_q + 2*k3_q + k4_q);
end
F = Function('F', {X0, U}, {X, Q}, {'x0','p'}, {'xf', 'qf'});
 
% Start with an empty NLP
w={};
w0 = [];
lbw = [];
ubw = [];
J = 0;
g={};
lbg = [];
ubg = [];
 
% Formulate the NLP
VirtualX=PositionX-(vv/ww)*(sin(Pose));
VirtualY=PositionY+(vv/ww)*(cos(Pose));
VirtualZ=Pose;
Xk= [VirtualX;VirtualY;VirtualZ];


for k=0:N-1
    % New NLP variable for the control
    Uk = MX.sym(['U_' num2str(k)]);
    w = {w{:}, Uk};
    lbw = [lbw; -0.5];
    ubw = [ubw;  0.5];
    w0 = [w0;  0];
 
    % Integrate till the end of the interval
    Fk = F('x0', Xk, 'p', Uk);
    Xk = Fk.xf;
    J=J+Fk.qf;

    for i = 1:verticesNum
       g = [g; {(Xk(1)+(vv/ww)*sin(Xk(3))-voronoiX(i))*(voronoiY(i+1)-voronoiY(i))-(Xk(2)-(vv/ww)*cos(Xk(3))-voronoiY(i))*(voronoiX(i+1)-voronoiX(i))}];
    end

 lbg=[lbg;lowbound];
 ubg=[ubg;upbound];      
end

%Create an NLP solver
prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
opts = struct;
opts.ipopt.print_level = 0;
opts.print_time = false;
solver = nlpsol('solver', 'ipopt', prob, opts);
% % Solve the NLP
sol = solver('x0', w0, 'lbx', lbw, 'ubx', ubw,...
             'lbg', lbg, 'ubg', ubg);
w_opt = full(sol.x);

u_opt = w_opt;
x_opt = [VirtualX;VirtualY;VirtualZ];
for k=0:N-1
    Fk = F('x0', x_opt(:,end), 'p', u_opt(k+1));
    x_opt = [x_opt, full(Fk.xf)];
end

x_opt = x_opt';

% origin position