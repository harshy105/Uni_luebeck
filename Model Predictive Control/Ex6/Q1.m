close all;
clear;
% start MPT
%tbxmanager restorepath;
%mpt_init;
% data initilisation
A = [1.5,1.2;0,1];
B = [0.6;0.4];
C = eye(2);
Q = eye(2);
R = 1;
Gx = [1 0; -1 0; 0 1; 0 -1];
hx = [5; 5; 5; 5];
Gu = [1; -1];
hu = [1; 1];
X = Polyhedron(Gx,hx); % MPT command
U = Polyhedron(Gu,hu);
K_inf = dlqr(A,B,Q,R,[]);

%% part b
% calculate the Presets (i)
pre1_X = inv(A-B*K_inf)*X;
pre2_X = (inv(A-B*K_inf))^2*X;
pre3_X = (inv(A-B*K_inf))^3*X;
% 
figure();
hold on;
plot(pre3_X,'color','m');
plot(pre2_X,'color','b');
plot(pre1_X,'color','g');
plot(X,'color','r')
title('pre set autonomous system (without control constraint)')
hold off;

%calculate the reach sets (ii)
reach1_X =  (A-B*K_inf)*X;
reach2_X = (A-B*K_inf)^2*X;
reach3_X = (A-B*K_inf)^3*X;

figure();
hold on;
plot(X,'color','r');
plot(reach1_X,'color','g');
plot(reach2_X,'color','b');
plot(reach3_X,'color','m')
title('reach set autonomous system(without control constraint)')
hold off;

%iii
disp('Since the Reach set are decreasing (or Pre sets are increasing)')
disp('i.e. all the state are driving to origin and hence the system is stable')

%iv
%X_ut = X.intersect(-(K_inf)'*U);
G = [Gx; -Gu*K_inf];
h = [hx; hu];
X_ut = Polyhedron(G,h);

pre1_X_ut = inv(A-B*K_inf)*X_ut;
pre2_X_ut = (inv(A-B*K_inf))^2*X_ut;
pre3_X_ut = (inv(A-B*K_inf))^3*X_ut;
reach1_X_ut = (A-B*K_inf)*X_ut;
reach2_X_ut = (A-B*K_inf)^2*X_ut;
reach3_X_ut = (A-B*K_inf)^3*X_ut;

figure();
hold on;
plot(pre3_X_ut,'color','m');
plot(pre2_X_ut,'color','b');
plot(pre1_X_ut,'color','g');
plot(X_ut,'color','r')
title('pre set autonomous system (control constraint)')
hold off;

figure();
hold on;
plot(X_ut,'color','r');
plot(reach1_X_ut,'color','g');
plot(reach2_X_ut,'color','b');
plot(reach3_X_ut,'color','m')
title('reach set autonomous system (control constraint)')
hold off;

%% Part C
% i
O_inf = MaxPosInvar(inv(A-B*K_inf),X_ut);

figure();
plot(O_inf);
title('O_\infty with LQR')

O_inf_vrep = O_inf.minVRep;

%ii already done in the previous exercise sheet, use MPC with x0 as
%diffenent vertices of the O_inf

%% part d
% pre set and reach set for a non autonomous case

pre1_X_nonauto = inv(A)*(X+(-B*U));
pre2_X_nonauto = inv(A)*(pre1_X_nonauto+(-B*U));
pre3_X_nonauto = inv(A)*(pre2_X_nonauto+(-B*U));

% P = X+(-B*U);
% G = P.A;
% h = P.b;
% pre1_X_nonauto = Polyhedron(G*A,h);
% P = pre1_X_nonauto+(-B*U);
% pre2_X_nonauto = Polyhedron(P.A *A, P.b);
% P = pre2_X_nonauto+(-B*U);
% pre3_X_nonauto = Polyhedron(P.A *A, P.b);

reach1_X_nonauto = A*X + B*U;
reach2_X_nonauto = A*reach1_X_nonauto + B*U;
reach3_X_nonauto = A*reach2_X_nonauto + B*U;

figure();
hold on;
plot(X,'color','r')
plot(pre1_X_nonauto,'color','g');
plot(pre2_X_nonauto,'color','b');
plot(pre3_X_nonauto,'color','m');
title('pre set (control invariant)')
hold off;

figure();
hold on;
plot(reach3_X_nonauto,'color','m')
plot(reach2_X_nonauto,'color','b');
plot(reach1_X_nonauto,'color','g');
plot(X,'color','r');
title('reach set (control invariant)')
hold off;

%% part e

C_inf = MaxConInvar(A,B,X,U);

figure();
plot(C_inf,'color','g');
hold on;
plot(O_inf,'color','r');
title('O_\infty with LQR and C_\infty');
legend('C_\infty','O_\infty');
hold off;

