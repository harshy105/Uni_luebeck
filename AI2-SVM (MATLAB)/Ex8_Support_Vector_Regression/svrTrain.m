function [alphas, alphas_dash, d] = svrTrain( X, y, K, epsilon, C )
% Input
% -----
%
% X          ... Data points.
%                [ x_11, x_12;
%                  x_21, x_22;
%                  x_31, x_32;
%                  ...              ]
%
% y          ... Class labels.
%                [ s_1; s_2; s_3; ... ]
%
% K          ... Kernel.
%                @(x, y) ...
%
% epsilon    ... SVR parameter.
%
%
% C          ... SVR parameter.

% Output
% ------
%
% alphas     ... Lagrange multipliers.
%
% alphas_dash... Lagrange multipliers.
%
% d          ... Distance from the origin.

% 1.    Fabian Domberg 
% 2.	Rakesh Reddy
% 3.	Tim-Henrik Traving
% 4.	Harsh Yadav

% YOUR IMPLEMENTATION GOES HERE...

m = length(y);

k = zeros(m,m);
for i = 1:m
    for j = 1:m
        k(i,j) = K(X(i,:),X(j,:));
    end
end
Q = [k -k; -k k];

c = zeros(2*m,1);
for i = 1:2*m
    if i<=m
        c(i) = epsilon - y(i);
    else
        c(i) = epsilon + y(i-m);
    end    
end

Aeq = [ones(1,m) -1*ones(1,m)];
beq = 0;
A = [-eye(2*m); eye(2*m)];
b = [zeros(2*m,1);C*ones(2*m,1)];

alphas_all = quadprog(Q,c,A,b,Aeq,beq);

alphas = alphas_all(1:m);
alphas_dash = alphas_all(m+1:2*m);

d_j = zeros(m,1);
for j = 1:m
    sum_d = 0;
    for i = 1:m
        sum_d = sum_d + (alphas(i)-alphas_dash(i))*K(X(i,:),X(j,:));        
    end
    if abs(alphas(j)) > exp(-8)
        d_j(j) = y(j) - sum_d - epsilon;
    elseif abs(alphas_dash(j)) > exp(-3)
        d_j(j) = y(j) - sum_d + epsilon;
    end
end
d_j_idx = d_j(abs(d_j)>exp(-8));
d = mean(d_j_idx,1);

end