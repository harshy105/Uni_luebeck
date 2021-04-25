function q = trajectory(q_start,q_end,ts,te,dt)

A = [1 te te^2 te^3 te^4 te^5
    1 ts ts^2 ts^3 ts^4 ts^5
    0 1 2*te 3*te^2 4*te^3 5*te^4
    0 1 2*ts 3*ts^2 4*ts^3 5*ts^4
    0 0 2 6*te 12*te^2 20*te^3
    0 0 2 6*ts 12*ts^2 20*ts^3];
B = [q_end;q_start;0 0;0 0;0 0;0 0];
a = A\B

t = ts:dt:te;

T = zeros(size(A,2),length(t));
for i = 1:size(A,2)    
    T(i,:) = t.^(i-1);
end

q = a'*T;
end
