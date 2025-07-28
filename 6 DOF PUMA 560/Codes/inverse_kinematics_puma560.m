function inverse_kinematics_puma560()
    %% Clear environment
    clc;

    %% Define symbolic variables for joint angles (real)
    syms th1 th2 th3 th4 th5 th6 real

    %% DH parameters: [a, alpha, d, theta]
    DH = [ ...
        0,       pi/2,     0,        th1;
        0.4318,  0,        0,        th2;
        0.0203, -pi/2,     0.15005,  th3;
        0,       pi/2,     0.4318,   th4;
        0,      -pi/2,     0,        th5;
        0,       0,        0,        th6];

    n = size(DH,1);

    %% Joint limits (Puma560 typical, radians)
    angle_limits = deg2rad([ ...
        -160, 160;   % th1
        -225, 45;   % th2
        -225, 45;   % th3
        -110, 170;   % th4
        -100, 100;   % th5
        -266, 266]); % th6

    %% Prompt user for target position
    target_pos = input('Enter target end-effector position as [x y z] in meters: ');
    target_pos = target_pos(:);

    %% Reachability check (simple)
    if ~isReachable(target_pos, DH)
        error('Target position is outside the robot''s reachable workspace. Please try another position.');
    end

    %% Solve IK
    try
        theta_sol = ikine_puma560(DH, target_pos, angle_limits);
    catch ME
        warning('No IK solution found for this target position. Message: %s', ME.message);
        return
    end

    %% Forward kinematics verification
    fprintf('\n--- Forward Kinematics Verification ---\n');
    L_a = DH(:,1);
    L_alpha = DH(:,2);
    L_d = DH(:,3);

    % Compute forward transformations
    [T0_i, ~] = get_T0i(theta_sol, L_a, L_d, L_alpha);

    % Extract end-effector position
    end_eff_pos = T0_i{end}(1:3,4);

    fprintf('Computed End-Effector Position: X=%.3f, Y=%.3f, Z=%.3f\n', ...
        end_eff_pos(1), end_eff_pos(2), end_eff_pos(3));
    fprintf('Target End-Effector Position:   X=%.3f, Y=%.3f, Z=%.3f\n', target_pos);

    %% Plot robot configuration
    plot_robot(T0_i);

    fprintf('\nInverse kinematics completed.\n');
    disp('Joint angles (deg):');
    disp(rad2deg(theta_sol'));
end

%% Inverse kinematics solver function (numeric, geometric, for position only)
function theta_sol = ikine_puma560(DH, target_pos, angle_limits)
    % Extract DH parameters
    a = DH(:,1); alpha = DH(:,2); d = DH(:,3);

    % For simplicity, do a geometric IK for position only (ignoring orientation)
    % This is a common approach for 6DOF arms with spherical wrists.
    x = target_pos(1); y = target_pos(2); z = target_pos(3);

    % Calculate wrist center position
    d6 = 0; % Puma560 has no offset at the end
    d4 = 0.4318;
    wx = x - d6; % For Puma560, d6 = 0
    wy = y - d6;
    wz = z - d6;

    % th1
    th1 = atan2(wy, wx);

    % r and s for planar calculation
    r = sqrt(wx^2 + wy^2);
    s = wz;

    % Link lengths
    a2 = 0.4318;
    a3 = 0.0203;
    d3 = 0.15005;
    d4 = 0.4318;

    % Compute th3 using cosine law
    D = (r^2 + (s - d3)^2 - a2^2 - d4^2) / (2 * a2 * d4);
    if abs(D) > 1
        error('No solution: point is outside reachable workspace.');
    end
    th3 = atan2(-sqrt(1 - D^2), D); % elbow-down

    % Compute th2
    phi1 = atan2(s - d3, r);
    phi2 = atan2(d4 * sin(th3), a2 + d4 * cos(th3));
    th2 = phi1 - phi2;

    % th4, th5, th6: For position-only IK, set to zero (or solve for orientation if needed)
    th4 = 0;
    th5 = 0;
    th6 = 0;

    % Assemble solution
    theta_sol = [th1; th2; th3; th4; th5; th6];

    % Check joint limits
    for i = 1:6
        if theta_sol(i) < angle_limits(i,1) || theta_sol(i) > angle_limits(i,2)
            error('Joint %d angle %.2f deg is out of range [%.2f, %.2f] deg.', ...
                i, rad2deg(theta_sol(i)), rad2deg(angle_limits(i,1)), rad2deg(angle_limits(i,2)));
        end
    end
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
    % Standard DH convention
    D = [cos(theta),             -sin(theta)*cos(alpha),    sin(theta)*sin(alpha),     a*cos(theta);
         sin(theta),              cos(theta)*cos(alpha),   -cos(theta)*sin(alpha),     a*sin(theta);
         0,                       sin(alpha),               cos(alpha),                d;
         0,                       0,                        0,                         1];
end

%% Plot robot joints and links
function plot_robot(T0_i)
    figure; hold on; grid on; axis equal;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('Puma560 Robot Configuration');

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

    axis([-0.5 0.8 -0.6 0.6 -0.3 1.0]);
    view(3);
end

%% Workspace reachability check
function reachable = isReachable(target_pos, DH)
    % Maximum reach is sum of link lengths (a parameters)
    max_reach = sum(DH(:,1)) + sum(DH(:,3));
    r = norm(target_pos(1:2)); % XY-plane distance
    z = target_pos(3);

    % Simple reachability check: within cylinder of radius max_reach and reasonable height
    min_z = min(DH(:,3)); % minimum d offset (could be negative)
    max_z = sum(DH(:,3)) + max(DH(:,1)); % rough upper bound

    reachable = (r <= max_reach) && (z >= min_z) && (z <= max_z);
end
