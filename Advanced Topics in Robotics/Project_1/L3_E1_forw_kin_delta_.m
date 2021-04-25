function [theta1, theta2, theta3, theta4, theta5, theta6, theta7] = L3_E1_forw_kin_delta_(delta,T07)
     % dimension of Kuka arm 
    d = [0.34 0 0.4 0 0.4 0 0.096]; 

    % wrist position wrt to base
    w = T07(1:3,4) - T07(1:3,3)*d(7); 

    %wrist position wrt joint 1
    s0 = [0;0;1]*d(1);

    sw = w - s0;
    x0s = sw/norm(sw);

    y0s = cross([0;0;1],x0s); 
    y0s = y0s/norm(y0s);

    z0s = cross(x0s,y0s);

    T0s = cat(2,x0s,y0s,z0s,s0);
    T0s = cat(1,T0s,[0 0 0 1]);

    %calculation of alpha1
    alpha1 = acos((d(3)^2+norm(sw)^2-d(5)^2)/(2*d(3)*norm(sw)));

    %calculation of T_s_4 (denoted as Es4)

    %rotation (delta) about x axis
    T_s_delta = [1 0 0 0   
        0 cos(delta) -sin(delta) 0
        0 sin(delta) cos(delta) 0
        0 0 0 1];
    % rotation (-alpha1) about y axis
    T_delta_aplha1 = [cos(alpha1) 0 -sin(alpha1) 0
        0 1 0 0
        sin(alpha1) 0 cos(alpha1) 0
        0 0 0 1];
    % translation (d(3)) along x axis
    T_alpha1_4 = [1 0 0 d(3)
        0 1 0 0
        0 0 1 0
        0 0 0 1];
    Ts4 = T_s_delta * T_delta_aplha1 * T_alpha1_4;

    % calculation of T04
    T04 = T0s*Ts4;

    %calculation of theta 1 and 2
    shoulder = [-1 1];
    theta1 = atan2(shoulder*T04(2,4),shoulder*T04(1,4))';    
    theta2 = atan2(shoulder*norm(T04(1:2,4)), T04(3,4)-d(1))';

    %calculation of theta 4
    elbow = [1 -1]; 
    alpha2 = acos((d(3)^2+d(5)^2-norm(sw)^2)/(2*d(3)*d(5)));
    theta4 = elbow*(pi-alpha2);

    theta4 = cat(1,theta4,-theta4); 

    %calculation of theta 3

    % calculation of position of wrist in origin
    origin = [1 0 0
        0 1 0 
        0 0 1];
    T0w = cat(2,origin,w);
    T0w = cat(1,T0w,[0 0 0 1]); 


    %wrist position after joint 1 tranformation
    T01 = [cos(theta1(1)) -sin(theta1(1)) 0 0 
        sin(theta1(1)) cos(theta1(1)) 0 0
        0 0 1 d(1)
        0 0 0 1];

    T1w = T01\T0w; 

    q = T1w(:,4);            

    s3 = -q(2)/(d(5)*sin(theta4(1,1))); % q_y = -d5*s3*s4
    c3 = sqrt(1-s3^2);

    theta3 = atan2(elbow*s3,elbow*c3);
    theta3 = cat(1,theta3,theta3);
    %calculation of Transformation matricies for the wrist position

    T02 = zeros(4,4,2); % Transformation matrix from base to shoulder (2 independent orientations)
    T23 = zeros(4,4,2); % Transformation martix from shoulder joint 3(depentdent on theta1)
    T34 = zeros(4,4,2,2); % Transformation martix from joint 3 to elbow(2 more independent orientations)
    T46 = zeros(4,4,2,2); % Transformation martix from elbow to wrist (dependent on theta1 and theta2 and theta3)
    T06 = zeros(4,4,2,2); % Transformation matrix from base to wrist (2*2=4 orientations)
    T67 = zeros(4,4,2,2); % Transformation martix from wrist to end effector (4 orientations)

    for i =1:2
        T02(:,:,i) = [cos(theta1(i)) -sin(theta1(i)) 0 0
            sin(theta1(i)) cos(theta1(i)) 0 0
            0 0 1 d(1)
            0 0 0 1];
        T23(:,:,i) = [cos(theta2(i)) 0 sin(theta2(i)) 0
                0 1 0 0
                -sin(theta2(i)) 0 cos(theta2(i)) 0
                0 0 0 1];   
        for j = 1:2
            T34(:,:,i,j) = [cos(theta3(i,j)) -sin(theta3(i,j)) 0 0
                sin(theta3(i,j)) cos(theta3(i,j)) 0 0
                0 0 1 d(3)
                0 0 0 1];          
            T46(:,:,i,j) = [cos(theta4(i,j)) 0 -sin(theta4(i,j)) -d(5)*sin(theta4(i,j))
                0 1 0 0
                sin(theta4(i,j)) 0 cos(theta4(i,j)) d(5)*cos(theta4(i,j))
                0 0 0 1];
            T06(:,:,i,j) = T02(:,:,i)*T23(:,:,i)*T34(:,:,i,j)*T46(:,:,i,j);
            T67(:,:,i,j) = T06(:,:,i,j)\T07;          
            fmt = ['position of wrist is: [', repmat('%g, ', 1, numel(T06(:,4,1,1))-1), '%g]\n'];
            fprintf(fmt, T06(:,4,1,1))
        end        
    end

    %calculation of theta 5, 6 and 7, from the fact that T67 is R_zyz
    theta5 = zeros(2,2,2);
    theta6 = zeros(2,2,2);
    theta7 = zeros(2,2,2);

    for i = 1:2
        for j = 1:2
            for k = 1:2

                if k == 1
                    hand = 1;
                    theta5(i,j,k) = atan2(hand*T67(2,3,i,j) , hand*T67(1,3,i,j));
                    theta6(i,j,k) = atan2(hand*sqrt(1-T67(3,3,i,j)^2),T67(3,3,i,j));
                    theta7(i,j,k) = atan2(hand*T67(3,2,i,j) , -hand*T67(3,1,i,j));
                else
                    hand = -1;
                    theta5(i,j,k) = atan2(hand*T67(2,3,i,j) , hand*T67(1,3,i,j));
                    theta6(i,j,k) = atan2(hand*sqrt(1-T67(3,3,i,j)^2),T67(3,3,i,j));
                    theta7(i,j,k) = atan2(hand*T67(3,2,i,j) , -hand*T67(3,1,i,j));                
                end

            end
        end
    end
end