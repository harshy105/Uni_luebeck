function theta = end_to_joint(q, l1, l2, elbow)

syms theta1 theta2

x = l1*cos(theta1)+l2*cos(theta1+theta2);
y = l1*sin(theta1)+l2*sin(theta1+theta2);

[theta1,theta2] = solve([x == q(1), y == q(2)],[theta1, theta2]);

theta = [double(theta1(elbow)),double(theta2(elbow))];
end