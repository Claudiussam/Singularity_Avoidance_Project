function forward_kinematics_5DOF()
    %% Clear and initialize
    clc;

    %% Define symbolic variables (only for reference, not used in numeric calc)
    syms th1 th2 th3 th5 th6

    %% Define DH parameters (a, d, alpha, theta)
    % Note: theta will be numeric, constructed from target_theta + offsets
    % Robot-specific DH parameters (example values)
    % Columns: a, d, alpha, theta (initially symbolic)
    DH_sym = [ ...
        0,    300,  -pi/2,   th1;    % Joint 1
        250,   0,    0,      th2;    % Joint 2
        160,   0,    0,      th3;    % Joint 3
        0,     0,   -pi/2,   th5;    % Joint 5
        0,     72,    0,     th6];   % Joint 6

    n = size(DH_sym,1);

    %% Define target joint angles (radians)
    % Example target angles for joints 1 to 5 (6th will be computed)
   fprintf('Enter joint angles in degrees for the 5 DOF robot.\n');
   fprintf('Example format: [-15, -30, 45, -90, 30]\n');
   target_theta_deg = input('Angles: ');  % User enters values like [-15, -30, 45, -90, 30]
   target_theta = target_theta_deg * pi/180;  % Convert to radians
   %% --- Joint limits ---
    theta_min = deg2rad([-150, -60, -110, -90, -200]);  % min limits
    theta_max = deg2rad([150, 120, 120, 90, 200]);         % max limits

    %% Check joint angles against limits
    for i = 1:length(target_theta)
        if target_theta(i) < theta_min(i) || target_theta(i) > theta_max(i)
            warning('Joint %d angle %.2f deg is out of range [%.2f, %.2f] deg.', ...
                i, rad2deg(target_theta(i)), rad2deg(theta_min(i)), rad2deg(theta_max(i)));
            % Uncomment below to clip instead of warning:
            target_theta(i) = max(min(target_theta(i), theta_max(i)), theta_min(i));
        end
    end
    %% Define angle offsets for each joint (motor angle - actual x-axis)
    angle_offset = [0, 90, 0, 0, 0] * pi/180; % Adjust as per your robot

    %% Build numeric DH table with offsets added to theta
    DH_num = zeros(n,4);
    for i = 1:n
        DH_num(i,1) = DH_sym(i,1); % a
        DH_num(i,2) = DH_sym(i,2); % d
        DH_num(i,3) = DH_sym(i,3); % alpha
        DH_num(i,4) = target_theta(i) + angle_offset(i); % theta + offset
    end

    %% Perform forward kinematics
    [T0_i, ~] = get_T0i(DH_num(:,4), DH_num(:,1), DH_num(:,2), DH_num(:,3));

    %% Extract joint positions
    joint_pos = cellfun(@(T) T(1:3,4), T0_i, 'UniformOutput', false);

    %% Print joint angles and end-effector position
    fprintf('Joint angles (degrees):\n');
    for i = 1:n
        fprintf('Theta%d: %.2f\n', i, rad2deg(DH_num(i,4)));
    end
    end_effector_pos = joint_pos{end};
    fprintf('\nEnd-effector position:\nX: %.3f, Y: %.3f, Z: %.3f\n', ...
        end_effector_pos(1), end_effector_pos(2), end_effector_pos(3));

    %% Plot the robot arm
    figure; hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('5 DOF Robot Forward Kinematics');

    % Plot base to first joint
    plot3([0, joint_pos{1}(1)], [0, joint_pos{1}(2)], [0, joint_pos{1}(3)], 'r-', 'LineWidth', 2);

    % Plot links between joints
    for i = 2:n
        plot3([joint_pos{i-1}(1), joint_pos{i}(1)], ...
              [joint_pos{i-1}(2), joint_pos{i}(2)], ...
              [joint_pos{i-1}(3), joint_pos{i}(3)], ...
              'b-', 'LineWidth', 2);
    end

    % Plot joints as points
    for i = 1:n
        plot3(joint_pos{i}(1), joint_pos{i}(2), joint_pos{i}(3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    end

    % Set axis limits (adjust as needed)
    axis([-300 300 -300 300 -300 300]);
    view(3);

    fprintf('\nForward kinematics calculation and plotting done.\n');
end

%% Helper function: findTh4
function theta6 = findth6(theta)
    % theta: vector of joint angles
    % theta6 = pi/2 - sum of theta(3) and theta(5)
    theta6 = pi/2 - sum(theta(3:5));
end

%% Helper function: get_T0i
function [T0_i, Tj_i] = get_T0i(theta, a, d, alpha)
    n = length(a);
    T0_i = cell(1,n);
    Tj_i = cell(1,n);
    T0_j = eye(4);
    for i = 1:n
        Tj_i{i} = forwardTransfer(a(i), alpha(i), d(i), theta(i));
        T0_i{i} = T0_j * Tj_i{i};
        T0_j = T0_i{i};
    end
end

%% Helper function: forwardTransfer
function D = forwardTransfer(a, alpha, d, theta)
    % Rotation about z by theta
    A = [cos(theta) -sin(theta) 0 0;
         sin(theta)  cos(theta) 0 0;
         0           0          1 0;
         0           0          0 1];

    % Translation along x by a and along z by d
    B = [1 0 0 a;
         0 1 0 0;
         0 0 1 d;
         0 0 0 1];

    % Rotation about x by alpha
    C = [1 0          0           0;
         0 cos(alpha) -sin(alpha) 0;
         0 sin(alpha)  cos(alpha) 0;
         0 0          0           1];

    D = A * B * C;
end
