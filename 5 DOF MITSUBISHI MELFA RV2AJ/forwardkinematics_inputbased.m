clear
clc

% Define symbolic variables for 5 DOF joints
syms th1 th2 th3 th5 th6

% Corrected DH parameters for 5 DOF (5 columns)
DH = [0, 13, 4.5, 11.75, 6.5;       % a (5 values)
      2.2, 0, 0, 0, 0;              % d (5 values)
      pi/2, 0, 0, 0, -pi/2;          % alpha (5 values)
      th1, th2, th3, th5, th6]';     % theta (5 values)

n = size(DH, 1); % Number of joints (now 5)

%% -------------------------------------------- Get user input for joint angles
fprintf('Enter joint angles in degrees for the 5 DOF robot.\n');
fprintf('Example format: [-15, -30, 45, -90, 30]\n');
target_theta_deg = input('Angles: ');  % User enters values like [-15, -30, 45, -90, 30]
target_theta = target_theta_deg * pi/180;  % Convert to radians

%% -------------------------------------------- Initializations
fprintf('Initializations\n')

L_a = DH(:, 1); 
L_d = DH(:, 2); 
L_alpha = DH(:, 3); 
L_theta = DH(:, 4);

%% -------------------------------------------- Forward Kinematic
fprintf('Forward Kinematic\n');
angle_offset = [0, 90, 0, 0, 0] * pi/180; % 5-element offset

% Use the 5DOF forward kinematics function
L_pos_sol = fkine5DOF(DH, target_theta, angle_offset);

%% -------------------------------------------- Numeric forward kinematics
fprintf('Numeric forward kinematics\n')
theta_num = subs(L_theta, symvar(L_theta), target_theta + angle_offset);
[T0_i, Tj_i] = get_T0i(theta_num, L_a, L_d, L_alpha);

% Extract joint positions
for i = 1:n
    joint_pos_num{i} = T0_i{i}(1:3, 4); 
end

%% -------------------------------------------- Prepare plot data
fprintf('Preparing plot data\n')
for i = 1:n
    X0_i{i} = joint_pos_num{i}(1);
    Y0_i{i} = joint_pos_num{i}(2);
    Z0_i{i} = joint_pos_num{i}(3);
end

%% -------------------------------------------- Plot
fprintf('Plotting\n')

figure
arm(1) = plot3([0 X0_i{1}], [0 Y0_i{1}], [0 Z0_i{1}], 'r', 'LineWidth', 2);
hold on

% Plot remaining segments
for i = 2:n
    arm(i) = plot3([X0_i{i-1} X0_i{i}],...
                   [Y0_i{i-1} Y0_i{i}],...
                   [Z0_i{i-1} Z0_i{i}],...
                   'Color', [mod(i,2) 0 mod(i+1,2)],...
                   'LineWidth', 2);
end

% Configure plot
xlim([-25 25])
ylim([-25 25])
zlim([-15 30])
grid on
title('5 DOF Robot Arm Configuration')
xlabel('X-axis')
ylabel('Y-axis')
zlabel('Z-axis')

%% -------------------------------------------- Display results
fprintf('\nJoint Angles (Degrees):\n');
for i = 1:n
    fprintf('Joint %d: %.2f°\n', i-1, target_theta_deg(i))
end

fprintf('\nEnd Effector Position:\n');
fprintf('X: %.2f | Y: %.2f | Z: %.2f\n\n', L_pos_sol(1), L_pos_sol(2), L_pos_sol(3));