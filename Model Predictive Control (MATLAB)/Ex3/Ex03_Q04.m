clear 
clc
A = [0,1
    0,-1];
Aeq = [1,0];

b = [sqrt(3/4)
    sqrt(3/4)];
beq = 1;

fun =@(x) x(2);
x0 = [0,0];

x = fmincon(fun,x0,A,b,Aeq,beq);