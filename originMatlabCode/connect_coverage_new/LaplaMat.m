function [L,A,d]=LaplaMat(x,R,deta)
%输入：x为N*n的状态矩阵，N为代理数量，n为状态维度；R为通信范围
%输出：n*n的拉普拉斯矩阵L与邻接矩阵A
value=-(R^2/log(deta));%2*sigma^2
d=squareform(pdist(x,'euclidean'));%距离矩阵
N=size(x,1);
A=zeros(N,N);
for i=1:N
    for j=1:i
        if i==j
            continue
        end
        if d(i,j)<=R
            A(i,j)=exp(-d(i,j)^2/(value));
        else
            A(i,j)=0;
        end
        A(j,i)=A(i,j); 
    end
end
A=A-diag(diag(A));%邻接矩阵
D=diag(sum(A,2));%度矩阵
L=D-A;%拉普拉斯矩阵
end