clear all;
close all;
clc;

A = [0.7115 -0.4345; 0.4345 0.8853];
B = [0.2173; 0.0573];
C = [0 1];

Gu = [1; -1];
hu = [5; 5];
Gx = [0 1; 0 -1];
hx = [100; 0];

Q = eye(2);
R = 1; 
N = 5;

[K_inf,P_inf,~] = dlqr(A,B,Q,R,0);
P_lyp = dlyap(A,Q);
P = P_inf;

x0 = [0; 6]; %for [0;10] the x2 will decrease below zero, hence problem is infeasible
ts = 0.1;
t_end = 5;
t = t_end/ts;

x_mpc = [x0, zeros(size(x0,1),t)];
x_lqr = [x0, zeros(size(x0,1),t)];
u_mpc = zeros(1,t);
u_lqr = zeros(1,t);
for i=1:t
    u_optimum = uoptimum(A,B,Gu,hu,Gx,hx,P,Q,R,N,x_mpc(:,i));
    u_mpc(i) = u_optimum(1);
    x_mpc(:,i+1) = A*x_mpc(:,i) + B*u_mpc(i); 
end

figure();
subplot(2,1,1);
plot(0:ts:(t_end-ts),u_mpc);
xlabel('N');
ylabel('Control input(U)');
title('Control input from MPC');

subplot(2,1,2);
plot(x_mpc(1,:),x_mpc(2,:));
xlabel('x_1');
ylabel('x_2');
title('phase plot from MPC');




