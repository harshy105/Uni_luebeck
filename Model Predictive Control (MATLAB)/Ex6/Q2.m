clear all; clc; close all;
% start MPT
%tbxmanager restorepath;
%mpt_init;

A = [1.2 1; 0 1];
B = [0; 1];
C = [1 0; 0 1];
Gu = [1; -1];
hu = [1; 1];
Gx = [1 0; -1 0; 0 1; 0 -1];
hx = [15; 15; 15; 15];

Q = [1 0; 0 1];
R = 1; 
N = 10;%3;

%% a
Gf = [1 0; -1 0; 0 1; 0 -1];
hf = [0; 0; 0; 0];
P = [1 0; 0 1];
% i
x0 = [2;-1];
x_open = [x0, zeros(size(x0,1),N)];
%open loop
u_optimum = uoptimum(A,B,Gu,hu,Gx,hx,Gf,hf,P,Q,R,N,x0);
for i=1:N
    x_open(:,i+1) = A*x_open(:,i) + B*u_optimum(i); 
end

figure();
subplot(2,1,1);
plot(x_open(1,:),x_open(2,:));
xlabel('x_1');
ylabel('x_2');
title('Open loop phase plot');
grid on;

%close loop MPC
u_mpc = zeros(1,N);
x_mpc = [x0, zeros(size(x0,1),N)];
for i=1:N
    u_optimum = uoptimum(A,B,Gu,hu,Gx,hx,Gf,hf,P,Q,R,N,x_mpc(:,i));
    u_mpc(i) = u_optimum(1);
    x_mpc(:,i+1) = A*x_mpc(:,i) + B*u_mpc(i);
end
subplot(2,1,2);
plot(x_mpc(1,:),x_mpc(2,:));
xlabel('x_1');
ylabel('x_2');
title('Close loop phase plot');
grid on;

% iii

%% b i

[K_inf,P_inf,~] = dlqr(A,B,Q,R,0);  
N = 3;
t = 4;
x0 = [2; -1];
u_mpc = zeros(1,t);
x_mpc = [x0, zeros(size(x0,1),t)];
for i=1:t
    u_optimum = uoptimum(A,B,Gu,hu,Gx,hx,Gf,hf,P,Q,R,N,x_mpc(:,i));
    u_mpc(i) = u_optimum(1);
    x_mpc(:,i+1) = A*x_mpc(:,i) + B*u_mpc(i);
end
figure();
plot(x_mpc(1,:),x_mpc(2,:));
xlabel('x_1');
ylabel('x_2');
title('Close loop phase plot');
grid on;




