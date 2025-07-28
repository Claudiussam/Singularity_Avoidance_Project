function forward_kinematics_puma560()
    %% Clear and initialize
    clc;

    %% Define symbolic variables (for reference)
    syms th1 th2 th3 th4 th5 th6

    %% Puma560 DH parameters (standard, in meters and radians)
    % [a,     alpha,      d,        theta]
    DH = [ ...
        0,      pi/2,     0,        th1;          % Joint 1
        0.4318, 0,        0,        th2;          % Joint 2
        0.0203, -pi/2,    0.15005,  th3;          % Joint 3
        0,      pi/2,     0.4318,   th4;          % Joint 4
        0,     -pi/2,     0,        th5;          % Joint 5
        0,      0,        0,        th6];         % Joint 6

    n = size(DH,1);

    %% Define target joint angles (radians)
    fprintf('Enter joint angles in degrees for the 6 DOF Puma560 robot.\n');
    fprintf('Example format: [0, 0, 0, 0, 0, 0]\n');
    target_theta_deg = input('Angles: ');  % e.g., [0, 0, 0, 0, 0, 0]
    if numel(target_theta_deg) ~= 6
        error('Please enter 6 joint angles.');
    end
    target_theta = target_theta_deg * pi/180;  % Convert to radians

    %% --- Joint limits (Puma560 typical) ---
    theta_min = deg2rad([-160, -225, -225, -110, -100, -266]);
    theta_max = deg2rad([160, 45, 45, 170, 100, 266]);

    %% Check joint angles against limits
    for i = 1:6
        if target_theta(i) < theta_min(i) || target_theta(i) > theta_max(i)
            warning('Joint %d angle %.2f deg is out of range [%.2f, %.2f] deg.', ...
                i, rad2deg(target_theta(i)), rad2deg(theta_min(i)), rad2deg(theta_max(i)));
            % Optionally clip:
            target_theta(i) = max(min(target_theta(i), theta_max(i)), theta_min(i));
        end
    end

    %% Build numeric DH table
    DH_num = DH;
    for i = 1:n
        DH_num(i,4) = target_theta(i); % theta
    end

    %% Perform forward kinematics
    [T0_i, ~] = get_T0i(DH_num(:,4), DH_num(:,1), DH_num(:,3), DH_num(:,2));

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
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('Puma560 Forward Kinematics');

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
    axis([-0.5 0.8 -0.6 0.6 -0.3 1.0]);
    view(3);

    fprintf('\nForward kinematics calculation and plotting done.\n');
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
    % Standard DH convention
    D = [cos(theta),             -sin(theta)*cos(alpha),    sin(theta)*sin(alpha),     a*cos(theta);
         sin(theta),              cos(theta)*cos(alpha),   -cos(theta)*sin(alpha),     a*sin(theta);
         0,                       sin(alpha),               cos(alpha),                d;
         0,                       0,                        0,                         1];
end
