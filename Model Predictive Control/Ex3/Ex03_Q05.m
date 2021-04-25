f = [0 0 -1]; % x = [x1, x2, r];

A = [2 3; 1 0; 1 -3; -1 -1; -1 0; 0 1];
b = [6 2 9 5 3 3]';

B = zeros(size(A,1),1);
for i = 1:size(A,1)
    B(i) = norm(A(i,:));
end

A = [A B];

x = linprog(f,A,b);