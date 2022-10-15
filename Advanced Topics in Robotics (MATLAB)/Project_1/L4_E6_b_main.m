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
    for i=1:1:(n+1)
        nameLink= sprintf('LBR_iiwa_7_R800_link%i',i);
        [returnCode,linkHandles(i)] = sim.simxGetObjectHandle(clientID,...
            nameLink,sim.simx_opmode_blocking);
        if returnCode < 0
            error('Can not find all link handles!');
        end
    end
 %% Set initial position
    Target_pos = zeros(n);
    for i=1:1:n
        returnCode = sim.simxSetJointTargetPosition(clientID,jointHandles(i),Target_pos(i),...
            sim.simx_opmode_blocking);
    end
    pause(1);
    
    %there is an offset in the end effector output, calculating the offset
    [~, euler_home] = sim.simxGetObjectOrientation(clientID, linkHandles(8),-1,sim.simx_opmode_blocking);
    [~, position_home] = sim.simxGetObjectPosition(clientID, linkHandles(8),-1,sim.simx_opmode_blocking);
        
    T_coppelia_home = L4_E6_transformation_Euler(euler_home,position_home);
    T_home = L4_E6_transformation_Euler([0 0 0],[0 0 1.2360]);
    T_calibrate = T_coppelia_home\T_home;
    
  %% Inverse Kinematics numerical method
  
    %end effector position and orientation
    end_pos = [0.5 -0.35 0.6];
    end_orient = [sqrt(1/2) 0 sqrt(1/2); 0 1 0; -sqrt(1/2) 0 sqrt(1/2)];
%     end_orient = [1 0 0; 0 1 0; 0 0 1];
    
    % transformation matrix of end effecter in base frame
    Tse = cat(2,end_orient,end_pos');
    Tse = cat(1,Tse,[0 0 0 1]);
    
    % dimension of Kuka arm 
    d = [0.34 0 0.4 0 0.4 0 0.096];
    
    %Tranformation matrix formulation
    poe = [0 0 1 0 0 0
        0 1 0 d(3)+d(5)+d(7) 0 0
        0 0 1 0 0 0
        0 -1 0 -d(5)-d(7) 0 0
        0 0 1 0 0 0
        0 1 0 d(7) 0 0
        0 0 1 0 0 0];
    B = poe';
    n = size(B,2);
    
    %initilization
    theta = [1 1 1 1 1 1 1]'; 
    error = 0.001;
    max_iter = 100;
    Tsb = L4_E6_for_kin(theta,d);
    Tbe = Tsb\Tse;
    vb = L4_E6_vb(Tbe);
        
    iter = 0;
    while (norm(vb(1:3))>error || norm(vb(4:6))>error) && (iter < max_iter) 
        iter = iter + 1;

        %calculation of jacobian
        J = zeros(size(B));
        J(:,n) = B(:,n);
        k = eye(4);

        for i = n-1:-1:1
            k = k * L4_E6_expo(B(:,i+1), -theta(i+1));
            r_k  = k(1:3,1:3);
            p_k = [0 -k(3,4) k(2,4)
                k(3,4) 0 -k(1,4)
                -k(2,4) k(1,4) 0];
            ad_k = [r_k zeros(3,3)
                p_k*r_k r_k];
            J(:,i) = ad_k * B(:,i);        
        end
        
        %update of theta
        theta = theta + 0.1.*pinv(J)*vb; 
        for i=1:7
            if theta(i)>pi
                theta(i) = theta(i) - 2*pi;
            elseif theta(i)<-pi
                theta(i) = theta(i) + 2*pi;
            end                
        end
               
       
        %run the simulation
        
        Target_pos = [theta(1) theta(2) theta(3) theta(4) theta(5) theta(6) theta(7)]; %theta1 pi/2-theta2(1) 0 theta4(1)
            for i=1:1:n
                returnCode = sim.simxSetJointTargetPosition(clientID,jointHandles(i),Target_pos(i),...
                    sim.simx_opmode_blocking);
            end
                
        % get the end effector euler angle and position        
        [~, euler_end] = sim.simxGetObjectOrientation(clientID, linkHandles(8),-1,sim.simx_opmode_blocking);
        [~, position_end] = sim.simxGetObjectPosition(clientID, linkHandles(8),-1,sim.simx_opmode_blocking);
        
        % calculation of the transformation matrix from the euler value and postion        
        T_coppelia_end = L4_E6_transformation_Euler(euler_end,position_end);
        
       %calibrating the transformation matrix from base to end effector
       %(check line 45 to 51 for the T_calibrate solution
        Tsb = T_coppelia_end*T_calibrate;
        
%         % Transfromation matrix from analytical solution 
%         Tsb_ana = L4_E6_for_kin(theta,d);       
        
        %calculating the tranformation matrix from current end effector
        %position to the final end position
        Tbe = Tsb\Tse;

        %update the vb       
        vb = L4_E6_vb(Tbe);
        
    end
    sim.simxFinish(-1); 
    
    
end