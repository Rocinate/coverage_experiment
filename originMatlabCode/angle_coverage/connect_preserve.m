function uc=connect_preserve(lamde2,x2,x,d,A,R,deta,Epsilon)
%输出：uc表示连通保持控制律，为nx2的向量，列分别对应x和y
%此控制律保持连通度不低于设定阈值。
%输入：lamde2表示nx1特征值估计向量，x2表示对应的特征向量估计nx1。
%x表示位置矩阵
%d表示距离平方矩阵nxn，A表示邻接矩阵，R表示通信半径，deta表示通信半径下的权值
%Epsilon表示连通度最小阈值。
value=-(R^2/log(deta))/2;%sigma^2
n=size(lamde2,1);
uc=zeros(n,2);
for i=1:n
    if lamde2(i)<= Epsilon
        a1=-50000;
    else
        a1=-10*csch(lamde2(i)-Epsilon)^2;
    end
    a2x=0;a2y=0;
    ind=find(d(i,:)<=R);%邻居集索引（包括自己）
    ind(ind==i)=[];%删去自己
    for j=1:size(ind,2)
        agent=ind(j);%邻居索引,边（i，agent）
        a2x=a2x+(-A(i,agent)/value)*(x(i,1)-x(agent,1))*(x2(i)-x2(agent))^2;%x分量
        a2y=a2y+(-A(i,agent)/value)*(x(i,2)-x(agent,2))*(x2(i)-x2(agent))^2;%y分量
    end
    ucx=-a1*a2x;
    ucy=-a1*a2y;
    uc(i,:)=[ucx,ucy];
end
end