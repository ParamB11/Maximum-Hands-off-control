function dxdt = maxh_dynamics(t,x)
dxdt = zeros(2,1); % x(1) refers to x ,x(2) refers to costate vector
u=0;
% Control law as found using maximum principle
if (x(2)>1)
    u=1;
elseif (-1<x(2)<1)
    u=0;
elseif (x(2)<-1)
    u=-1;
elseif (x(2)==1)
    u=0;
elseif (x(2)==-1)
    u=0;
end
dxdt(1)= x(1)^2 + u;
dxdt(2)= -2*x(1)*x(2); 
end