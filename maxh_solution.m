clear; clc;
%% Problem Data and variables
T=10;
tol = 0.001; % tolerance should be greater than 0.0002
x0 = -0.1;
a = 0.1; b = 0.5; % Found such that x_a*x_b<=0
c = 0;
x_a = -0.05; x_b = 0.1085;
x_c = 0;
next_iter = true;

%% Bisection method
while next_iter 
    c = (a+b)/2 ;
    [t_v,x_v] = ode23t(@(t,x) maxh_dynamics(t,x), [0 T], [x0 c]);
    x_c = x_v(length(x_v),1);
    if(x_c*x_a <= 0)
        b = c;
        x_b = x_c;
    else 
        a = c;
        x_a = x_c;
    end
    if (abs(x_c)>tol)
        next_iter = true;
    else 
        next_iter = false;
    end
end

t_final = length(x_v);
x_final = x_v(t_final,1);
u = zeros(t_final,1); % Lp optimal control

for i=1:t_final
   if (x_v(i,2)>1)
    u(i)=1;
   elseif (-1<x_v(i,2)<1)
    u(i)=0;
   elseif (x_v(i,2)<-1)
    u(i)=-1;
   elseif (x_v(i,2)==1)
    u(i)=0;
   elseif (x_v(i,2)==-1)
    u(i)=0;
   end 
end

% Ploting the results
figure('Name','State');
plot(t_v, x_v(:,1));
figure ('Name','Costate');
plot(t_v, x_v(:,2));
figure('Name','Control Input');
plot(t_v, u);