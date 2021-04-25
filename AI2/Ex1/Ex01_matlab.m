% Submission from:
% 1.	Fabian Domberg 
% 2.	Rakesh Reddy
% 3.	Tim-Henrik Traving
% 4.	Harsh Yadav

clc;clear;
A = [1,1; 5,2; 1,0; 0,1; -1,0; 0,-1];
b = [10,30,6,9,0,0];
f = [-30,-25];
x = linprog(f,A,b);