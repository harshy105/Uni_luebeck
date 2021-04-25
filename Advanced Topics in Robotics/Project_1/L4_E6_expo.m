function e = L4_E6_expo(Bi, theta_i)

%calculates: exp(-[B_i]*theta_i)

wi = [0 -Bi(3) Bi(2)
    Bi(3) 0 -Bi(1)
    -Bi(2) Bi(1) 0];
vi = Bi(4:6);

e_w_theta_i = eye(3) + sin(theta_i).*wi + (1-cos(theta_i)).*(wi*wi);

g_theta_i = eye(3).*theta_i + (1-cos(theta_i)).*wi + (theta_i-sin(theta_i)).*(wi*wi);

e = [e_w_theta_i g_theta_i*vi
    0 0 0 1];

end