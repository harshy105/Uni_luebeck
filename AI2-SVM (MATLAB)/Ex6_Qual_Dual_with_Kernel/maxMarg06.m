function [alphas, idx] = maxMarg06( X, y, K )
% Input
% -----
%
% X        ... Data points and class labels.
%              [ x_11, x_12;
%                x_21, x_22;
%                x_31, x_32;
%                ...              ]
%
% y        ... Class labels.
%              [ s_1, s_2, s_3, ... ]
%
% K        ... Kernel.
%              @(x, y) ...

% Output
% ------
%
% alphas   ... Lagrange multipliers.
%
% idx      ... Indices of non-zero alphas.

% 1.    Fabian Domberg 
% 2.	Rakesh Reddy
% 3.	Tim-Henrik Traving
% 4.	Harsh Yadav

% YOUR IMPLEMENTATION GOES HERE...

n = size(X,1);
H_d = zeros(n,n);
for i = 1:n
    for j = 1:n
        H_d(i,j) = (y(i)*y(j)).*K(X(i,:),X(j,:));
    end
end
f_d = -1.*ones(n,1);
A_d= zeros(n,n);
b_d = zeros(n,1);
Aeq_d = y;
beq_d = 0;
lb = zeros(n,1);

alphas = quadprog(H_d,f_d,A_d,b_d,Aeq_d,beq_d,lb);
idx = abs(alphas) > exp(-10);


end