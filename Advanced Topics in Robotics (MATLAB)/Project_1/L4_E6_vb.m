function vb = L4_E6_vb(T)

R = T(1:3,1:3);
p = T(1:3,4);
%calculates: log(Tranfomation matrix) = R(6,1)

if abs(R(1,1)-1)<0.0001 && abs(R(2,2)-1)<0.0001 && abs(R(3,3)-1)<0.0001
    w = [0;0;0];
    if norm(p) == 0 
        theta = 0;
        v = [0;1;0];
    else
        theta = norm(T(1:3,4));
        v = T(1:3,4)/theta;
    end   
    
else

    if abs(trace(T))<0.00001 % or trace(R) = -1
        theta = pi;
        w_skew = T(1:3,1:3)+eye(3);
        w_skew(:,1) = (1/sqrt(2*(1+T(1,1)))).*w_skew(:,1);
        w_skew(:,2) = (1/sqrt(2*(1+T(2,2)))).*w_skew(:,2);
        w_skew(:,3) = (1/sqrt(2*(1+T(3,3)))).*w_skew(:,3);

    else
        theta = acos((trace(R)-1)/2);
        w_skew = (R-R')/(2*sin(theta));
    end

    ginv_theta = eye(3,3)./theta - w_skew./2 + (w_skew*w_skew).*(1/theta - cot(theta/2)/2);
    v = ginv_theta*p;   
    w = [w_skew(3,2)
        w_skew(1,3)
        w_skew(2,1)];
end

vb = cat(1,w.*theta,v.*theta);


end