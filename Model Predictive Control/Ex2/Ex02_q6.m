A = [-0.4 -1.1 0 
    4 5 0
    0 0 0.9];
B = [0;1;0];
C = [1 1 0];

%a)
eigvalue = eig(A);
%b)
Co = ctrb(A,B);
kc = rank(Co);
%c)
Hautus_stab = rank([4*eye(3)-A, B]);
%d)
Ob = obsv(A,C);
ko = rank(Ob);
%e)
Hautus_detect = rank([4*eye(3)-A', C']);

