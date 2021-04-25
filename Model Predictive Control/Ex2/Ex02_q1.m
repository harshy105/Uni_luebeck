clc;clear;
%% 1a
alpha = 0;
A = [1/3 0 0 0
    0 -1/2 alpha 0
    0 1/2 -5/4 0
    -1/2 0 0 1/3];
B = [0;-2;4;0];
C = [1/2 0 1/3 0];
Co = ctrb(A,B);
Ob = obsv(A,C);
kc = rank(Co);
ko = rank(Ob);

%slabilizablity

alpha = 1/2;
A = [1/3 0 0 0
    0 -1/2 alpha 0
    0 1/2 -5/4 0
    -1/2 0 0 1/3];

%apply similarity transfromation page 144 of the slides

[e_vec, e_value] = eig(A);

A_transformed = e_vec\A*e_vec;
B_transformed = e_vec\B;

% from A_transformed it can be seen that only 1st element in unstable and
% from B_transfromed it can be seen we can control on 1st element 
% it means that the system is stabilizable
% same can be checked with Hautus test below

rank_hautus = rank([4*eye(size(A,1))-A B]);

