function [Cx,Cy] = PolyCentroid(X,Y)
Xa = [X(2:end);X(1)];
Ya = [Y(2:end);Y(1)];

A = 1/2*sum(X.*Ya-Xa.*Y); %signed area of the polygon

Cx = (1/(6*A)*sum((X + Xa).*(X.*Ya-Xa.*Y)));
Cy = (1/(6*A)*sum((Y + Ya).*(X.*Ya-Xa.*Y)));