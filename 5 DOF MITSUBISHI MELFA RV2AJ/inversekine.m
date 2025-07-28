function L_motor_sol = ikine5DOF_v2(DH, target_pos, angle_range_motor, angle_offset)
%% -------------------------------------------- Initializations
L_a = DH(:,1);   % Arm
L_d = DH(:,2);   % Offset
L_alpha = DH(:,3); 
L_theta = DH(:,4); % Joint angles: θ₁, θ₂, θ₃, θ₅, θ₆ (no θ₄)

n = length(L_a);  % Number of joints (5)

%% Handle optional inputs
if ~exist('angle_offset', 'var')
    angle_offset = zeros(1,5); % 5-element offset [θ₁, θ₂, θ₃, θ₅, θ₆]
end
if ~exist('angle_range_motor', 'var')
    % Joint limits for θ₁, θ₂, θ₃, θ₅, θ₆ (example values)
    angle_range_motor = [-150 150; -60 120; -110 120; -90 90; -200 200] * pi/180; 
end

% Adjust angle ranges with offset
angle_range_theta = [angle_range_motor(:,1) + angle_offset',...
                     angle_range_motor(:,2) + angle_offset'];

%% -------------------------------------------- Symbolic forward kinematics
fprintf('Symbolic forward kinematics\n');
[T0_i, ~] = get_T0i(L_theta, L_a, L_d, L_alpha);
joint_pos_sym = T0_i{end}(1:3,4); % End-effector position (includes θ₅ and θ₆)

%% -------------------------------------------- Inverse kinematics
fprintf('Solving inverse kinematics\n');

% Equation: End-effector position matches target
syms θ₁ θ₂ θ₃ θ₅ θ₆
eq = (joint_pos_sym == target_pos); 

% Substitute θ₅ based on kinematic relationship (e.g., θ₅ = f(θ₂, θ₃))
theta_5_expr = pi/2 - (theta_2 + theta_3); % Example dependency (adjust based on your robot)
eq = subs(eq, theta_5, theta_5_expr);

% Solve for θ₁, θ₂, θ₃, θ₆
variables = [theta_1, theta_2, theta_3, theta_6]; % Solve 4 variables (underdetermined system)
sol = vpasolve(eq, variables, angle_range_theta(1:4,:));

% Extract solutions
theta_num = [double(sol.theta_1), double(sol.theta_2), double(sol.theta_3), double(sol.theta_6)];
theta_num = [theta_num, pi/2 - (theta_num(2) + theta_num(3))]; % Compute θ₅

%% -------------------------------------------- Validate solutions
valid = true;
for i = 1:5
    if theta_num(i) < angle_range_theta(i,1) || theta_num(i) > angle_range_theta(i,2)
        valid = false;
        break;
    end
end

if ~valid
    error('No valid solution within joint limits.');
end

%% -------------------------------------------- Compute motor angles
L_theta_sol = theta_num';
L_motor_sol = L_theta_sol - angle_offset'; % Adjust for offsets

end