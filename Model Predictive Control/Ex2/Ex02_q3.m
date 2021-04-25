clc;clear;

% 3.a.i: Euler
A = [-5 2.7
    -3.1 1.5];
B = [4 2.1
    1.1 3];
C = [1 1];
D = zeros(size(C,1),size(B,2));
ts = 1;

A1 = eye(size(A,1))+ts.*A;
B1 = ts.*B;

% 3.a.ii: Exact
A2 = expm(ts.*A);
B2 = A\(A2-eye(size(A,1)))*B;

% 3.b: Ct to dt by matlab
ts = 0.1;
ct = ss(A,B,C,D);
dt = c2d(ct,ts);

% 3.c: plot the response
t = 0:ts:10;
u = [sin(t);
    cos(t)];

lsim(dt,u);
hold on;
lsim(ct,u,t);
hold off;
grid on;
