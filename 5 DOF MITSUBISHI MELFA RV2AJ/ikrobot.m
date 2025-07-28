clear
clc

%% Define symbolic variables PROPERLY
syms th1 th2 th3 th5 th6 real  % Correct symbolic declaration

%% DH parameters for 5 DOF
DH = [0   13    4.5   11.75  6.5;   % a
      2.2 0     0      0      3;     % d
      pi/2 0    0      0     -pi/2;  % alpha
      th1  th2  th3    th5    th6]'; % theta

target_pos = [-4 -5 25];

%% -------------------------------------------- Initializations
fprintf('Initializations\n');
L_a = DH(:,1);
L_d = DH(:,2);
L_alpha = DH(:,3); 
L_theta = DH(:,4);
n = size(DH,1);

%% -------------------------------------------- Inverse Kinematics
angle_offset = [0 90 0 0 0] * pi/180;
angle_range_motor = [-100 100; -75 75; -125 95; -150 150; -90 90] * pi/180;

L_motor_sol = ikine5DOF_v2(DH, target_pos, angle_range_motor, angle_offset);

%% -------------------------------------------- Numeric Forward Kinematics
fprintf('Numeric forward kinematics\n');
L_theta_sol = L_motor_sol + angle_offset';

[T0_i, ~] = get_T0i(L_theta_sol, L_a, L_d, L_alpha);

%% ... (rest of plotting code remains unchanged)
