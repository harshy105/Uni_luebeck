clear;clc;close all;
%tbxmanager restorepath;
%mpt_init;

A = [1.5 1.2; 0 1];
B = [0.6;0.4];
C = [1 0; 0 1];

model = LTISystem('A',A,'B',B,'C',C);
model.x.min = [-5; -5];
model.x.max = [5; 5];
model.u.min = -1;
model.u.max = 1;
Q = eye(2);
model.x.penalty = QuadFunction(Q);
R = 1;
model.u.penalty = QuadFunction(R);

N = 3;
mpc1 = MPCController(model,N);
expmpc1 = mpc1.toExplicit();
figure()
expmpc1.partition.plot();
title('critical regions');
figure()
expmpc1.feedback.fplot();
title('control input');
figure()
expmpc1.cost.fplot();
title('cost function');

%c 
x = [3; -1];
X = [x, zeros(size(x,1),N)];
T = repmat(zeros(2,N+1),12,1);
for k=1:12
    [u, feasible, openloop] = mpc1.evaluate(x); 
    T((k-1)*2+1:k*2,1) = x;
    x = A*x + B*u;
    X(1:2,k+1) = x;
    for i = 2:N+1
        T((k-1)*2+1:k*2,i) = A*T((k-1)*2+1:k*2,i-1) + B*openloop.U(1,i-1);
    end
end

figure();
subplot(2,1,1);
hold on;
for k = 1:12
    plot(T((k-1)*2+1,:),T((k*2),:)) 
end
plot(X(1,:),X(2,:));
xlabel('x_1');
ylabel('x_2');
title('Open loop phase plot');
grid on;
hold off;

%d 

% the max control invariant set is a subset of the J*, since there is no
% end cost matrix

%e

Tset = model.LQRSet;
P = model.LQRPenalty;
model.x.with('terminalSet');
model.x.terminalSet = Tset;
model.x.with('terminalPenalty');
model.x.terminalPenalty = P;
N = 3;
mpc2 = MPCController(model,N);
expmpc2 = mpc2.toExplicit();
figure()
expmpc2.partition.plot();
title('critical regions');
figure()
expmpc2.feedback.fplot();
title('control input');
figure()
expmpc2.cost.fplot();
title('cost function');

% f
% the maximum control invariant set is subset of J* since the the system
% has penatly on the terminal state, which makes it recursive feasible
