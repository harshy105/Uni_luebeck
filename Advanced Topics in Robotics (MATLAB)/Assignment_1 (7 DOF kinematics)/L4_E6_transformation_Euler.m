function [T] = L4_E6_transformation_Euler(euler,pos)

a = euler(1);
b = euler(2);
g = euler(3);


Rx = [1 0 0
    0 cos(a) -sin(a)
    0 sin(a) cos(a)];

Ry = [cos(b) 0 sin(b)
    0 1 0
    -sin(b) 0 cos(b)];

Rz = [cos(g) -sin(g) 0
    sin(g) cos(g) 0
    0 0 1];

T = [Rz*Ry*Rx pos'
    0 0 0 1];

end