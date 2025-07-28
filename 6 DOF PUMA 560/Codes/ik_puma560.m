function theta_sol = inverse_kinematics_puma560(target_pos)
    % target_pos: [x; y; z] (end-effector position)
    % Returns theta_sol: [theta1, theta2, theta3, theta4, theta5, theta6] in radians
% Example usage (uncomment these lines in your script)
  target_pos = [0.3; 0.1; 0.5]; % Replace with your desired [x; y; z]
   

    %% DH parameters: [a, alpha, d, theta]
    DH = [ ...
        0,       pi/2,     0,        0;
        0.4318,  0,        0,        0;
        0.0203, -pi/2,     0.15005,  0;
        0,       pi/2,     0.4318,   0;
        0,      -pi/2,     0,        0;
        0,       0,        0,        0];

    %% Joint limits (Puma560 typical, radians)
    angle_limits = deg2rad([ ...
        -160, 160;   % th1
        -225, 45;    % th2
        -225, 45;    % th3
        -110, 170;   % th4
        -100, 100;   % th5
        -266, 266]); % th6

    %% Reachability check (simple)
    if ~isReachable(target_pos, DH)
        error('Target position is outside the robot''s reachable workspace. Please try another position.');
    end

    %% Solve IK (call the helper function)
    try
        theta_sol = ikine_puma560(DH, target_pos, angle_limits);
    catch ME
        warning('No IK solution found for this target position. Message: %s', E.message);
        theta_sol = nan(6,1);
        return
    end

    %% Forward kinematics verification
    fprintf('\n--- Forward Kinematics Verification ---\n');
    L_a = DH(:,1);
    L_alpha = DH(:,2);
    L_d = DH(:,3);

    % Compute forward transformations
    [T0_i, ~] = get_T0i(theta_sol, L_a, L_d, L_alpha);
    
    % Extract joint positions
    joint_pos = cell(1, length(T0_i));
    for i = 1:length(T0_i)
        joint_pos{i} = T0_i{i}(1:3, 4);
    end

    % Extract end-effector position
    end_eff_pos = joint_pos{end};

    fprintf('Computed End-Effector Position: X=%.3f, Y=%.3f, Z=%.3f\n', ...
        end_eff_pos(1), end_eff_pos(2), end_eff_pos(3));
    fprintf('Target End-Effector Position:   X=%.3f, Y=%.3f, Z=%.3f\n', target_pos);

    %% Plot robot configuration
    plot_robot(joint_pos);

    fprintf('\nInverse kinematics completed.\n');
    disp('Joint angles (deg):');
    disp(rad2deg(theta_sol'));
end

function theta_sol = ikine_puma560(DH, target_pos, angle_limits)
    % Extract DH parameters
    a = DH(:,1); alpha = DH(:,2); d = DH(:,3);

    % For simplicity, do a geometric IK for position only (ignoring orientation)
    x = target_pos(1); y = target_pos(2); z = target_pos(3);

    % Calculate wrist center position (adjust for d4)
    d4 = 0.4318; % From joint 4's d parameter
    wx = x - d4 * 0; % Adjust based on orientation (simplified here)
    wy = y - d4 * 0; % Assuming no orientation offset for simplification
    wz = z - d4;

    % th1
    th1 = atan2(wy, wx);

    % r and s for planar calculation
    r = sqrt(wx^2 + wy^2);
    s = wz;

    % Link lengths
    a2 = 0.4318; % From joint 2's a parameter
    a3 = 0.0203; % From joint 3's a parameter
    d3 = 0.15005; % From joint 3's d parameter

    % Compute th3 using cosine law
    D = (r^2 + (s - d3)^2 - a2^2 - a3^2 - d4^2) / (2 * a2 * sqrt(a3^2 + d4^2));
    if abs(D) > 1
        error('No solution: point is outside reachable workspace.');
    end
    th3 = atan2(-sqrt(1 - D^2), D); % elbow-down

    % Compute th2
    phi1 = atan2(s - d3, r);
    phi2 = atan2(d4, a3) + atan2(sqrt(a3^2 + d4^2) * sin(th3), a2 + sqrt(a3^2 + d4^2) * cos(th3));
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

function reachable = isReachable(target_pos, DH)
    % Compute the maximum reach (sum of link lengths and offsets)
    max_reach = sum(abs(DH(:,1))) + sum(abs(DH(:,3)));
    r = norm(target_pos(1:2)); % XY-plane distance
    z = target_pos(3);

    % Simple check: within a sphere of radius max_reach
    reachable = (r^2 + (z)^2) <= (max_reach)^2;
end

function [T0_i, Tj_i] = get_T0i(theta, a, d, alpha)
    n = length(a);
    T0_i = cell(1,n);
    Tj_i = cell(1,n);
    T0_j = eye(4);
    for i = 1:n
        Tj_i{i} = dh_transform(a(i), alpha(i), d(i), theta(i));
        T0_i{i} = T0_j * Tj_i{i};
        T0_j = T0_i{i};
    end
end

function T = dh_transform(a, alpha, d, theta)
    T = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha),  -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end

function plot_robot(joint_pos)
    figure; hold on; grid on; axis equal;
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('Puma560 Inverse Kinematics');

    % Plot base to first joint
    plot3([0, joint_pos{1}(1)], [0, joint_pos{1}(2)], [0, joint_pos{1}(3)], 'r-', 'LineWidth', 2);

    % Plot links between joints
    for i = 2:length(joint_pos)
        plot3([joint_pos{i-1}(1), joint_pos{i}(1)], ...
              [joint_pos{i-1}(2), joint_pos{i}(2)], ...
              [joint_pos{i-1}(3), joint_pos{i}(3)], ...
              'b-', 'LineWidth', 2);
    end

    % Plot joints as points
    for i = 1:length(joint_pos)
        plot3(joint_pos{i}(1), joint_pos{i}(2), joint_pos{i}(3), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    end

    % Set axis limits
    axis([-1 1 -1 1 0 1.5]);
    view(3);
end

% Example usage (should be in a separate script or command window)
% target_pos = [0.5; 0; 0.5];
% theta_sol = inverse_kinematics_puma560(target_pos);
% assignin('base', 'theta1', theta_sol(1));
% assignin('base', 'theta2', theta_sol(2));
% assignin('base', 'theta3', theta_sol(3));
% assignin('base', 'theta4', theta_sol(4));
% assignin('base', 'theta5', theta_sol(5));
% assignin('base', 'theta6', theta_sol(6));
% sim('PUMA560.slx');