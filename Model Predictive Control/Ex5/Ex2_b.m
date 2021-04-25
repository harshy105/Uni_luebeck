clear all;
close all;
clc;

A = [1 1; 0 1];
B = [0; 1];
C = [1 0];

Gu = [1; -1];
hu = [1; 1];
Gx = [1 0; 0 1; -1 0; 0 -1];
hx = [15; 15; 15; 15];

Q = [1 0; 0 1];
R = 1; 
N = 10;
P = [1 0; 0 1];

x0 = [1; 1];
x = [x0, zeros(size(x0,1),N)];
u = zeros(1,N);
u_optimum = uoptimum(A,B,Gu,hu,Gx,hx,P,Q,R,N,x0);
for i=1:N
    x(:,i+1) = A*x(:,i) + B*u_optimum(i); 
end

figure();
subplot(2,2,1);
plot(u_optimum);
xlabel('N');
ylabel('Control input(U)');
title('Control input');

subplot(2,2,2);
plot(x(1,:),x(2,:));
xlabel('x_1');
ylabel('x_2');
title('phase plot');

x0 = [-1; -1];
x = [x0, zeros(size(x0,1),N)];
u = zeros(1,N);
u_optimum = uoptimum(A,B,Gu,hu,Gx,hx,P,Q,R,N,x0);
for i=1:N
    x(:,i+1) = A*x(:,i) + B*u_optimum(i); 
end

subplot(2,2,3);
plot(u_optimum);
xlabel('N');
ylabel('Control input(U)');
title('Control input');

subplot(2,2,4);
plot(x(1,:),x(2,:));
xlabel('x_1');
ylabel('x_2');
title('phase plot');

% ii 
figure();
x01 = -3:0.6:3;
x02 = x01;
U = zeros(length(x01),length(x02));
for a = 1:length(x01)
    for b = 1:length(x02)
        x0 = [x01(a); x02(b)];
        x = [x0, zeros(size(x0,1),N)];
        u_optimum = uoptimum(A,B,Gu,hu,Gx,hx,P,Q,R,N,x0);
        for i=1:N
            x(:,i+1) = A*x(:,i) + B*u_optimum(i); 
        end       
        plot(x(1,:),x(2,:));
        hold on;
        U(a,b) = u_optimum(1);
    end
end
xlabel('x_1');
ylabel('x_2');
title('phase plot');
hold off;

figure();
surf(x01,x02,U);
xlabel('x_1');
ylabel('x_2');
title('u_0*');
