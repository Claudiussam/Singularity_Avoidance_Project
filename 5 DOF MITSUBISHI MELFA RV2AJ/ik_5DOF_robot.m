function ik_5DOF_robot()
    %% Clear environment
    clc;

    %% Define symbolic variables for joint angles (real)
    syms th1 th2 th3 th5 th6 real

    %% DH parameters: [a, d, alpha, theta]
    DH = [ ...
        0,    300,  -pi/2,   th1;    % Joint 1
        250,   0,    0,      th2;    % Joint 2
        160,   0,    0,      th3;    % Joint 3
        0,     0,   -pi/2,   th5;    % Joint 5
        0,     72,    0,     th6];   % Joint 6

    n = size(DH,1);

    %% Angle offsets and joint limits (radians)
    angle_offset = deg2rad([0, 90, 0, 0, 0]);
    angle_range_motor_deg = [ ...
        -150, 150;  % th1
        -60,  120;   % th2
        -110, 120;   % th3
        -90,   90;  % th5
        -200, 200];  % th6
    angle_range_motor = deg2rad(angle_range_motor_deg);

    %% Prompt user for target position
    target_pos = input('Enter target end-effector position as [x y z]: ');
    target_pos = target_pos(:);

    %% Check reachability
    if ~isReachable(target_pos, DH)
        error('Target position is outside the robot''s reachable workspace. Please try another position.');
    end

    %% Try to solve IK with error handling
    try
        L_motor_sol = ikine5DOF_v2(DH, target_pos, angle_range_motor, angle_offset);
    catch ME
        warning('No IK solution found for this target position. Message: %s', ME.message);
        return
    end

    %% Forward kinematics verification
    fprintf('\n--- Forward Kinematics Verification ---\n');
    L_a = DH(:,1);
    L_d = DH(:,2);
    L_alpha = DH(:,3);

    % Add offsets to motor solution to get actual joint angles
    L_theta_sol = L_motor_sol + angle_offset';

    % Compute forward transformations
    [T0_i, ~] = get_T0i(L_theta_sol, L_a, L_d, L_alpha);

    % Extract end-effector position
    end_eff_pos = T0_i{end}(1:3,4);

    fprintf('Computed End-Effector Position: X=%.3f, Y=%.3f, Z=%.3f\n', ...
        end_eff_pos(1), end_eff_pos(2), end_eff_pos(3));
    fprintf('Target End-Effector Position:   X=%.3f, Y=%.3f, Z=%.3f\n', target_pos);

    %% Plot robot configuration
    plot_robot(T0_i);

    fprintf('\nInverse kinematics completed.\n');
end

%% Inverse kinematics solver function
function L_motor_sol = ikine5DOF_v2(DH, target_pos, angle_range_motor, angle_offset)
    L_a = DH(:,1);
    L_d = DH(:,2);
    L_alpha = DH(:,3);
    L_theta = DH(:,4);

    %% Symbolic FK
    [T0_i, ~] = get_T0i(L_theta, L_a, L_d, L_alpha);
    joint_pos_sym = T0_i{end}(1:3,4);

    %% Symbolic variables for solution
    syms th1 th2 th3 th5 th6 real

    %% th5 as function of th2 and th3
    theta_5_expr = pi/2 - (th2 + th3);

    %% Substitute th5
    joint_pos_sub = subs(joint_pos_sym, th5, theta_5_expr);

    %% IK equations
    eqns = [joint_pos_sub(1) == target_pos(1), ...
            joint_pos_sub(2) == target_pos(2), ...
            joint_pos_sub(3) == target_pos(3)];

    %% Variables to solve: th1, th2, th3, th6
    vars = [th1, th2, th3, th6];

    %% vpasolve with search ranges (lower and upper bounds)
    search_ranges = angle_range_motor([1,2,3,5], :); % th1, th2, th3, th6
    sol = vpasolve(eqns, vars, search_ranges);

    th1_sol = double(sol.th1);
    th2_sol = double(sol.th2);
    th3_sol = double(sol.th3);
    th6_sol = double(sol.th6);

    if isempty(th1_sol)
        error('No solution found for inverse kinematics.');
    end

    th5_sol = pi/2 - (th2_sol + th3_sol);
    solutions = [th1_sol, th2_sol, th3_sol, th5_sol, th6_sol];

    %% Filter solutions by joint limits
    valid_idx = [];
    for i = 1:size(solutions,1)
        sol_i = solutions(i,:);
        if all(sol_i >= angle_range_motor(:,1)' & sol_i <= angle_range_motor(:,2)')
            valid_idx(end+1) = i; %#ok<AGROW>
        end
    end

    if isempty(valid_idx)
        error('No valid IK solution within joint limits.');
    end

    chosen_sol = solutions(valid_idx(1), :);

    %% Adjust for angle offsets
    L_theta_sol = chosen_sol';
    L_motor_sol = L_theta_sol - angle_offset';
end

%% Forward kinematics helper function
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

%% DH forward transformation for one joint
function D = forwardTransfer(a, alpha, d, theta)
    A = [cos(theta), -sin(theta), 0, 0;
         sin(theta),  cos(theta), 0, 0;
         0,           0,          1, 0;
         0,           0,          0, 1];

    B = [1, 0, 0, a;
         0, 1, 0, 0;
         0, 0, 1, d;
         0, 0, 0, 1];

    C = [1, 0,           0,          0;
         0, cos(alpha), -sin(alpha), 0;
         0, sin(alpha),  cos(alpha), 0;
         0, 0,           0,          1];

    D = A * B * C;
end

%% Plot robot joints and links
function plot_robot(T0_i)
    figure; hold on; grid on; axis equal;
    xlabel('X'); ylabel('Y'); zlabel('Z');
    title('5 DOF Robot Configuration');

    base_pos = [0;0;0];
    joint_pos = cellfun(@(T) T(1:3,4), T0_i, 'UniformOutput', false);

    % Plot base to first joint
    plot3([base_pos(1), joint_pos{1}(1)], [base_pos(2), joint_pos{1}(2)], [base_pos(3), joint_pos{1}(3)], 'r-', 'LineWidth', 2);

    % Plot links between joints
    for i = 2:length(joint_pos)
        plot3([joint_pos{i-1}(1), joint_pos{i}(1)], ...
              [joint_pos{i-1}(2), joint_pos{i}(2)], ...
              [joint_pos{i-1}(3), joint_pos{i}(3)], ...
              'b-', 'LineWidth', 2);
    end

    % Plot joints
    for i = 1:length(joint_pos)
        plot3(joint_pos{i}(1), joint_pos{i}(2), joint_pos{i}(3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    end

    axis([-300 300 -300 300 -300 300]);
    view(3);
end

%% Workspace reachability check
function reachable = isReachable(target_pos, DH)
    % Maximum reach is sum of link lengths (a parameters)
    max_reach = sum(DH(:,1));
    r = norm(target_pos(1:2)); % XY-plane distance
    z = target_pos(3);

    % Simple reachability check: within cylinder of radius max_reach and reasonable height
    min_z = min(DH(:,2)); % minimum d offset (could be negative)
    max_z = sum(DH(:,2)) + max(DH(:,1)); % rough upper bound

    reachable = (r <= max_reach) && (z >= min_z) && (z <= max_z);
end
