clear; clc;
%% 1 a
a = rand(1,5);
b = rand(1,5);

%% 1 b
c = a*b';
A = a'*b;

%% 1 c
e = a.*b;

%% 1 d
element_1 = A(1,2);
element_2 = A(2,3);

%% 1 e
Con_row = cat(1,A(1,:),A(length(A),:)); % row concatenate
Con_column = cat(2,A(1,:),A(length(A),:)); % column concatenate

%% 1 f
A(A<0.5) = 0;

%% 1 g
B = magic(3);
for i = 1:length(B)
    for j = 1:length(B)
        if (i ~= j)
            B(i,j) = 0;
        end
    end
end
%or 
B = diag(diag(magic(3)));

%% 1 h
f = [1;2;3];
X = B\f; % shorted representation of: inv(B)*f

%% 1 i
eig_B = eig(B);

%% 2 a
data = load('mydata.txt');
x = data(1:2,:)';
y = data(3,:)';

gscatter(x(:,1),x(:,2),y,'br','o*');

%% 2 b
%classification using bruteforce
u1 = [4, 7];
[v1, class_u1] = bruteForce(x, y, u1);

u2 = [7, 5];
[v2, class_u2] = bruteForce(x, y, u2);

%classification using K-D search
class_kd_u1 = y(knnsearch(x,u1));
class_kd_u2 = y(knnsearch(x,u2));

%result from bruteFroce adn K-D search are same