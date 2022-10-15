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
    
    % end effector position and orientation
    end_pos = [0.5 0.4 0.5];
    end_orient = [sqrt(1/2) 0 sqrt(1/2); 0 1 0; -sqrt(1/2) 0 sqrt(1/2)];
%     end_pos = [0.7 0 0.34+0.096];
%     end_orient = [1 0 0; 0 1 0; 0 0 1];
    
    % transformation matrix of end effecter in base frame
    T07 = cat(2,end_orient,end_pos');
    T07 = cat(1,T07,[0 0 0 1]);
 
    %% varying delta
     
    for delta = -pi/4:0.1:pi/4 
        [theta1, theta2, theta3, theta4, theta5, theta6, theta7] = L3_E1_forw_kin_delta_(delta,T07);
        Target_pos = [theta1(2) theta2(2) theta3(2,1) theta4(2,1) theta5(2,1,2) theta6(2,1,2) theta7(2,1,2)];
        
            for l=1:1:n
                returnCode = sim.simxSetJointTargetPosition(clientID,jointHandles(l),Target_pos(l),...
                    sim.simx_opmode_blocking);                 
            end
            pause(1);
    end
%     
    %% 8 orientation
    
    delta = pi/4;
    [theta1, theta2, theta3, theta4, theta5, theta6, theta7] = L3_E1_forw_kin_delta_(delta,T07);
    for i = 1:2
        for j = 1:2
            for k = 1:2
                
                Target_pos = [theta1(i) theta2(i) theta3(i,j) theta4(i,j) theta5(i,j,k) theta6(i,j,k) theta7(i,j,k)];
                
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