%tbxmanager restorepath;
%mpt_init;
z = sdpvar(1,1);
x = sdpvar(2,1);
f = 0.5*z(1)^2 + 2*x(1)*z + x(2)^2;
C = [z(1)<=1+x(1) , -z(1)<=1-x(2) , -5<=x(1)<= 5 , -5<=x(2)<=5];

mpQP = Opt(C,f,x,z);
solution = mpQP.solve();

figure(); solution.xopt.plot();
figure(); solution.xopt.fplot('primal'); %z*
figure(); solution.xopt.fplot('obj'); %f*