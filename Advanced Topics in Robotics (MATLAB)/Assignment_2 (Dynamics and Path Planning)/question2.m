close all;
clear all;
clc;

%% a
m1 = 1; L1 = 1; r1 = 0.4; Jz1 = 0.2;
m2 = 0.5; L2 = 0.5; r2 = 0.2; Jz2 = 0.05;
q_start = [1.2,0];
q_end = [0.5,1];
t_start = 0;
t_end = 5;
elbow = 1; %1 or 2
dt = 0.1;

% linear trajectory calculation in Task space
q_a= trajectory(q_start,q_end,t_start,t_end,dt); 

%trajectory calculation in Joint space
theta_a = zeros(size(q_a,1),size(q_a,2));
for i = 1:size(q_a,2)
    theta_a(:,i) = end_to_joint(q_a(:,i)', L1, L2, elbow)';
end

% %% b
% % calculation of start and end value in Joint space
% theta_start = end_to_joint(q_start, L1, L2, elbow);
% theta_end = end_to_joint(q_end, L1, L2, elbow);
% 
% % linear trajectory calculation in Joint Space
% theta_b = trajectory(theta_start,theta_end,t_start,t_end,dt);
% 
% %trajectory calculation in Task space
% q_b = zeros(size(theta_b,1),size(theta_b,2));
% for i =1:size(theta_b,2)
%     q_b(1,i) = L1*cos(theta_b(1,i))+ L2*cos(theta_b(1,i)+theta_b(2,i));
%     q_b(2,i) = L1*sin(theta_b(1,i))+ L2*sin(theta_b(1,i)+theta_b(2,i));
% end
% 
% %plot 
% figure();
% plot(q_a(1,:),q_a(2,:));
% hold on;
% plot(q_b(1,:),q_b(2,:));
% xlabel('tip_x');
% ylabel('tip_y');
% legend('part a','part b');
% title('Trajectory of Tip end in Task Space')
% hold off;
% 
% figure();
% plot(theta_a(1,:),theta_a(2,:));
% hold on;
% plot(theta_b(1,:),theta_b(2,:));
% xlabel('theta_1');
% ylabel('theta_2');
% legend('part a','part b');
% title('Trajectory of Joint angles in Joint Space') 
% hold off;
% 
% %% c
% 
% % calculating Torque from the inverse dynamics for part a and b
% [tau_a, dtheta_a, d2theta_a] = inverse_dynamics(theta_a,dt);
% [tau_b, dtheta_b, d2theta_b] = inverse_dynamics(theta_b,dt);
% 
% t = t_start:dt:t_end;
% figure();
% subplot(2,1,1);
% plot(t(1:size(tau_a,2)),tau_a);
% xlabel('Time');
% ylabel('Torques');
% legend('Joint_1','Joint_2');
% title('Torques in part a');
% 
% subplot(2,1,2);
% plot(t(1:size(theta_a,2)),theta_a(1,:));
% hold on;
% plot(t(1:size(theta_a,2)),theta_a(2,:));
% plot(t(1:size(dtheta_a,2)),dtheta_a(1,:));
% plot(t(1:size(dtheta_a,2)),dtheta_a(2,:));
% plot(t(1:size(d2theta_a,2)),d2theta_a(1,:));
% plot(t(1:size(d2theta_a,2)),d2theta_a(1,:));
% xlabel('Time');
% legend('theta_1','theta_2','d(theta_1)','d(theta_2)','d2(theta_1)','d2(theta_2)');
% title('Joint Variable for a');
% hold off;
% 
% figure();
% subplot(2,1,1);
% plot(t(1:size(tau_b,2)),tau_b);
% xlabel('Time');
% ylabel('Torques');
% legend('Joint_1','Joint_2');
% title('Torques in part b');
% 
% subplot(2,1,2);
% plot(t(1:size(theta_b,2)),theta_b(1,:));
% hold on;
% plot(t(1:size(theta_b,2)),theta_b(2,:));
% plot(t(1:size(dtheta_b,2)),dtheta_b(1,:));
% plot(t(1:size(dtheta_b,2)),dtheta_b(2,:));
% plot(t(1:size(d2theta_b,2)),d2theta_b(1,:));
% plot(t(1:size(d2theta_b,2)),d2theta_b(1,:));
% xlabel('Time');
% legend('theta_1','theta_2','d(theta_1)','d(theta_2)','d2(theta_1)','d2(theta_2)');
% title('Joint Variable for b');
% hold off;
% 
% %% d
% 
% % Simulation of thetas the Joint Space for part a and b
% dtheta_start_a = dtheta_a(:,1);
% theta_a_simulation = forward_dynamics(tau_a,theta_start,dtheta_start_a,dt);
% 
% dtheta_start_b = dtheta_b(:,1);
% theta_b_simulation = forward_dynamics(tau_b,theta_start,dtheta_start_b,dt);
% 
% 
% figure();
% subplot(1,2,1);
% plot(theta_a(1,:),theta_a(2,:));
% hold on;
% plot(theta_b(1,:),theta_b(2,:));
% xlabel('theta_1');
% ylabel('theta_2');
% legend('part a','part b');
% title('Original Trajectory in Joint Space') 
% hold off;
% 
% subplot(1,2,2);
% plot(theta_a_simulation(1,:),theta_a_simulation(2,:));
% hold on;
% plot(theta_b_simulation(1,:),theta_b_simulation(2,:));
% xlabel('theta_1');
% ylabel('theta_2');
% legend('part a',' part b');
% title('Simulated Trajectory in Joint Space') 
% hold off;
% 
% % Simulation of Tip End in Task space for part a and b
% q_a_simulation = zeros(size(theta_a_simulation,1),size(theta_a_simulation,2));
% for i =1:size(theta_a_simulation,2)
%     q_a_simulation(1,i) = L1*cos(theta_a_simulation(1,i))+ L2*cos(theta_a_simulation(1,i)+theta_a_simulation(2,i));
%     q_a_simulation(2,i) = L1*sin(theta_a_simulation(1,i))+ L2*sin(theta_a_simulation(1,i)+theta_a_simulation(2,i));
% end
% 
% q_b_simulation = zeros(size(theta_b_simulation,1),size(theta_b_simulation,2));
% for i =1:size(theta_b_simulation,2)
%     q_b_simulation(1,i) = L1*cos(theta_b_simulation(1,i))+ L2*cos(theta_b_simulation(1,i)+theta_b_simulation(2,i));
%     q_b_simulation(2,i) = L1*sin(theta_b_simulation(1,i))+ L2*sin(theta_b_simulation(1,i)+theta_b_simulation(2,i));
% end
% 
% figure();
% subplot(1,2,1);
% plot(q_a(1,:),q_a(2,:));
% hold on;
% plot(q_b(1,:),q_b(2,:));
% xlabel('tip_x');
% ylabel('tip_y');
% legend('part a','part b');
% title('Original Trajectory in Task Space')
% axis([0.45,1.25,-0.5,1.5]);
% hold off;
% 
% subplot(1,2,2);
% plot(q_a_simulation(1,:),q_a_simulation(2,:));
% hold on;
% plot(q_b_simulation(1,:),q_b_simulation(2,:));
% xlabel('tip_x');
% ylabel('tip_y');
% legend('part a',' part b');
% title('Simulated Trajectory in Task Space') 
% axis([0.45,1.25,-0.5,1.5]);
% hold off;
% 
% 
% 
% 
% 
