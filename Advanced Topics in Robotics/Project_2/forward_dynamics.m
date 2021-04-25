function theta = forward_dynamics(tau,theta_start,dtheta_start,dt)

m1 = 1; L1 = 1; r1 = 0.4; Jz1 = 0.2;
m2 = 0.5; r2 = 0.2; Jz2 = 0.05;
alpha = Jz1 + Jz2 + m1*r1^2 + m2*r2^2 + m2*L1^2; 
beta = m2*L1*r2;
delta = Jz2 + m2*r2^2;
g = 9.81;

theta = zeros(2,length(tau)+1);
dtheta = zeros(2,length(tau)+1);
theta(:,1) = theta_start';
dtheta(:,1) = dtheta_start;

for i = 1:length(tau)
    M = [alpha+2*beta*cos(theta(2,i)), delta+beta*cos(theta(2,i))
        delta + beta*cos(theta(2,i)), delta];
    c = [-2*beta*sin(theta(2,i))*dtheta(1,i)*dtheta(2,i)-beta*sin(theta(2,i))*dtheta(2,i)^2
        beta*sin(theta(2,i))*dtheta(1,i)^2];
    G = [g*m2*(r2*cos(theta(1,i)+theta(2,i)) + L1*cos(theta(1,i))) + g*m1*r1*cos(theta(1,i))
        g*m2*r2*cos(theta(1,i)+theta(2,i))];
    d2theta = pinv(M)*(tau(:,i) - c - G);
    theta(:,i+1) = theta(:,i) + dtheta(:,i)*dt; 
    dtheta(:,i+1) = dtheta(:,i) + d2theta*dt;
end
end