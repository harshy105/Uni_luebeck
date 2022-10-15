% Clear all
clear all
close all
clc

% Add path for the API scripts
addpath('API')


%% Connect
sim=remApi('remoteApi');      % Creates remote API object which is called vrep
sim.simxFinish(-1);           % Closes any currently open connections
 
clientID = sim.simxStart('127.0.0.1',19999,true,true,5000,5);   % Creates a connection 
if (clientID>-1)
    disp('Connected')
    % Get joint handles
    n = 7; % robot has 7 joints
    jointHandles = zeros(n,1);
    linkHandles = zeros(n+1,1);
    for i=1:1:n
        nameJoint = sprintf('LBR_iiwa_7_R800_joint%i',i);
        [returnCode,jointHandles(i)] = sim.simxGetObjectHandle(clientID,...
            nameJoint,sim.simx_opmode_blocking);
        if returnCode < 0
            error('Can not find all joint handles!');
        end
    end
 %% Set initial position
    Target_pos = zeros(n);
    for i=1:1:n
        returnCode = sim.simxSetJointTargetPosition(clientID,jointHandles(i),Target_pos(i),...
            sim.simx_opmode_blocking);
%         pause(0.2);
    end
    pause(1);
    
    %% Inverse Kinematics
  
    %end effector position and orientation
    end_pos = [0.4 0.65 0.5];
    end_orient = [sqrt(1/2) 0 sqrt(1/2); 0 1 0; -sqrt(1/2) 0 sqrt(1/2)];
    
    % transformation matrix of end effecter in base frame
    T07 = cat(2,end_orient,end_pos');
    T07 = cat(1,T07,[0 0 0 1]);
    
    % dimension of Kuka arm 
    d = [0.34 0 0.4 0 0.4 0 0.096]; 
    
    % wrist position wrt to base
    w = T07(1:3,4) - T07(1:3,3)*d(7); 
    
    %wrist position wrt joint 1
    q = w - [0;0;1]*d(1);
    
    %calculation of theta1
    hand = [1 -1]; % for both postive and negative theta values
    theta1 = atan2(hand.*q(2),hand.*q(1)); 
       
    %calculation of theta4, given theta3 = 0 for Kuka Arm 
    cos_phi = (d(3)^2 + d(5)^2 - norm(q)^2) / (2*d(3)*d(5));
    
    theta4 = atan2(hand*sqrt(1-cos_phi^2),-cos_phi);
    
    %calculation of theta2  
    psi = atan2(q(3), norm(q(1:2)));
    
    omega = atan2(d(5)*sin(theta4), d(3) + d(5)*cos(theta4));
    
    theta2 = pi/2-(psi-omega);
    
    %arranging the orienation of theta 1,2 4 to go get same wrist position
    theta2 = cat(1,theta2,-theta2);
    
    theta4 = cat(1,theta4,-theta4);
    
    %calculation of Transformation matricies for the wrist position
    
    T02 = zeros(4,4,2); % Transformation matrix from base to shoulder (2 independent orientations)
    T24 = zeros(4,4,2,2); % Transformation martix from shoulder to elbow (2 more independent orientations)
    T46 = zeros(4,4,2,2); % Transformation martix from elbow to wrist (dependent on theta1 and theta2)
    T06 = zeros(4,4,2,2); % Transformation matrix from base to wrist (2*2=4 orientations)
    T67 = zeros(4,4,2,2); % Transformation martix from wrist to end effector (4 orientations)
    
    for i =1:2
        T02(:,:,i) = [cos(theta1(i)) -sin(theta1(i)) 0 0
            sin(theta1(i)) cos(theta1(i)) 0 0
            0 0 1 d(1)
            0 0 0 1];
        for j = 1:2
            T24(:,:,i,j) = [cos(theta2(i,j)) 0 sin(theta2(i,j)) d(3)*sin(theta2(i,j))
                0 1 0 0
                -sin(theta2(i,j)) 0 cos(theta2(i,j)) d(3)*cos(theta2(i,j))
                0 0 0 1];            
            T46(:,:,i,j) = [cos(theta4(i,j)) 0 -sin(theta4(i,j)) -d(5)*sin(theta4(i,j))
                0 1 0 0
                sin(theta4(i,j)) 0 cos(theta4(i,j)) d(5)*cos(theta4(i,j))
                0 0 0 1];
            T06(:,:,i,j) = T02(:,:,i)*T24(:,:,i,j)*T46(:,:,i,j);
            T67(:,:,i,j) = T06(:,:,i,j)\T07;
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
      

    %% Forward Kinematics
    for i = 1:2
        for j = 1:2
            for k = 1:2
                
                Target_pos = [theta1(i) theta2(i,j) 0 theta4(i,j) theta5(i,j,k) theta6(i,j,k) theta7(i,j,k)];
                for l=1:1:n
                    returnCode = sim.simxSetJointTargetPosition(clientID,jointHandles(l),Target_pos(l),...
                        sim.simx_opmode_blocking);                       
                end
                pause(1);
                
            end
        end
    end
    
    sim.simxFinish(-1); 
    
end