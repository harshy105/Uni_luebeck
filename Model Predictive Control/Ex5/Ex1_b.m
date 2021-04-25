clear all;
close all;
clc;

A = [0.7115 -0.4345; 0.4345 0.8853];
B = [0.2173; 0.0573];
C = [0 1];

Gu = [1; -1];
hu = [5; 5];
Gx = [0 1; 0 -1];
hx = [100; 100];

Q = [100 0; 0 100];
R = 1; 
N = 2;

% i
[K_inf,P_inf,~] = dlqr(A,B,Q,R,0);
P_lyp = dlyap(A,Q);
eig(P_inf);
eig(P_lyp);

% ii-vi
P = P_inf;
x0 = [0; 10]; 
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
    
    u_lqr(i) = -K_inf*x_lqr(:,i);
    x_lqr(:,i+1) = A*x_lqr(:,i) + B*u_lqr(i);    
end

figure();
subplot(2,2,1);
plot(0:ts:(t_end-ts),u_mpc);
xlabel('N');
ylabel('Control input(U)');
title('Control input from MPC');

subplot(2,2,2);
plot(x_mpc(1,:),x_mpc(2,:));
xlabel('x_1');
ylabel('x_2');
title('phase plot from MPC');

subplot(2,2,3);
plot(0:ts:(t_end-ts),u_lqr);
xlabel('N');
ylabel('Control input(U)');
title('Control input from LQR');

subplot(2,2,4);
plot(x_lqr(1,:),x_lqr(2,:));
xlabel('x_1');
ylabel('x_2');
title('phase plot from LQR');



