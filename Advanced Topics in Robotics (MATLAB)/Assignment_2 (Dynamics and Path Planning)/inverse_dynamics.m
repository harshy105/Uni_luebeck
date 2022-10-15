function [tau, dtheta, d2theta]= inverse_dynamics(theta,dt)

m1 = 1; L1 = 1; r1 = 0.4; Jz1 = 0.2;
m2 = 0.5; r2 = 0.2; Jz2 = 0.05;
alpha = Jz1 + Jz2 + m1*r1^2 + m2*r2^2 + m2*L1^2; 
beta = m2*L1*r2;
delta = Jz2 + m2*r2^2;
g = 9.81;

%calculation of the derivative
dtheta = diff(theta,1,2) / dt;
d2theta = diff(dtheta,1,2) / dt;
tau = zeros(size(d2theta,1),size(d2theta,2));

for i = 1:size(d2theta,2)
    M = [alpha+2*beta*cos(theta(2,i)), delta+beta*cos(theta(2,i))
        delta + beta*cos(theta(2,i)), delta];
    c = [-2*beta*sin(theta(2,i))*dtheta(1,i)*dtheta(2,i)-beta*sin(theta(2,i))*dtheta(2,i)^2
        beta*sin(theta(2,i))*dtheta(1,i)^2];
    G = [g*m2*(r2*cos(theta(1,i)+theta(2,i)) + L1*cos(theta(1,i))) + g*m1*r1*cos(theta(1,i))
        g*m2*r2*cos(theta(1,i)+theta(2,i))];
    tau(:,i) = M*d2theta(:,i) + c + G;
end
end