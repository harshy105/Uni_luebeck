function T = L4_E6_for_kin(t,d)

%calculates the end effector position by POE

M1 = [cos(t(1)) -sin(t(1)) 0 0 
    sin(t(1)) cos(t(1)) 0 0
    0 0 1 d(1)
    0 0 0 1];
M2 = [cos(t(2)) 0 sin(t(2)) 0
    0 1 0 0
    -sin(t(2)) 0 cos(t(2)) 0
    0 0 0 1];
M3 = [cos(t(3)) -sin(t(3)) 0 0 
    sin(t(3)) cos(t(3)) 0 0
    0 0 1 d(3)
    0 0 0 1];
M4 = [cos(t(4)) 0 -sin(t(4)) 0
    0 1 0 0
    sin(t(4)) 0 cos(t(4)) 0
    0 0 0 1];
M5 = [cos(t(5)) -sin(t(5)) 0 0 
    sin(t(5)) cos(t(5)) 0 0
    0 0 1 d(5)
    0 0 0 1];
M6 = [cos(t(6)) 0 sin(t(6)) 0
    0 1 0 0
    -sin(t(6)) 0 cos(t(6)) 0
    0 0 0 1];
M7 = [cos(t(7)) -sin(t(7)) 0 0 
    sin(t(7)) cos(t(7)) 0 0
    0 0 1 d(7)
    0 0 0 1];
T = M1*M2*M3*M4*M5*M6*M7;
end