function f = svrProduce( X, K, alphas, alphas_dash, d, x1, x2 )
% Input
% -----
%
% X          ... Data points.
%                [ x_11, x_12;
%                  x_21, x_22;
%                  x_31, x_32;
%                  ...              ]
%
% K          ... Kernel.
%                @(x, y) ...
%
% alphas     ... Lagrange multipliers.
%
% alphas_dash... Lagrange multipliers.
%
% d          ... Distance from the origin.
%
% x1         ... Domain of x1, e.g. [-3 -2 -1 0 1 2 3].
%
% x2         ... Domain of x2, e.g. [-3 -2 -1 0 1 2 3].

% Output
% ------
%
% f          ... Approximated values of f(x) on the domain(s) of x1 and x2.

% 1.    Fabian Domberg 
% 2.	Rakesh Reddy
% 3.	Tim-Henrik Traving
% 4.	Harsh Yadav

% YOUR IMPLEMENTATION GOES HERE...

if ~isempty(x2)
    f = zeros(length(x1),length(x2));
    for l = 1:length(x1)
        for m = 1:length(x2)
            sum = 0;
            for i = 1:length(alphas)
                sum = sum + (alphas(i)-alphas_dash(i))*K(X(i,:),[x1(l),x2(m)]); 
            end
            f(l,m) = sum + d;
        end
    end
else
    f = zeros(length(x1),1);
    for l = 1:length(x1)
        sum = 0;
        for i = 1:length(alphas)
            sum = sum + (alphas(i)-alphas_dash(i))*K(X(i,:),x1(l)); 
        end
        f(l) = sum + d;    
    end
end

end