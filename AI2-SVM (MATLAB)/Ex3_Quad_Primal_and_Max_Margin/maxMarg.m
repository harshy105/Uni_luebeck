function [exitflag, w, d, margin, dists] = maxMarg( X, y )

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
%              [ s_1; s_2; s_3; ... ]

% Output
% ------
%
% exitflag ... Exitflag of quadprog.
%
% w        ... Weight vector.
%
% d        ... Distance from the origin.
%
% margin   ... Margin.
%
% dists    ... Distances of each data point to the separating plane.

% 1.    Fabian Domberg 
% 2.	Rakesh Reddy
% 3.	Tim-Henrik Traving
% 4.	Harsh Yadav

% YOUR IMPLEMENTATION GOES HERE...

m = size(X,1);
n = size(X,2);
X = cat(2,X,ones(m,1));
A = cat(1,-X((y==1),:),X((y==-1),:));
b = -1.*ones(m,1);
H = eye(n+1);
H(n+1,n+1) = 0;
f = zeros(n+1,1);

[x,~,exitflag] = quadprog(H,f,A,b);
margin = 1/norm(x(1:2));
w = margin.*x(1:2);
d = margin.*x(3);
dists = margin.*(X*x);


end