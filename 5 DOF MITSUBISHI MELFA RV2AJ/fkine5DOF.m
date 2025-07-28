function L_pos_sol = fkine5DOF(DH, theta, angle_offset)

%% -------------------------------------------- Initializations
L_a = DH(:,1); % Arm
L_d = DH(:,2); % Offset
L_alpha = DH(:,3); 
L_theta = DH(:,4);

n = length(L_a); % Number of joints (5 for 5 DOF)

%% Substitute theta values if provided
if exist('theta', 'var')
    L_theta = subs(L_theta, symvar(L_theta), theta);
end

%% Set default angle_offset if not provided
if nargin < 3
    angle_offset = zeros(1,5); % Default to [0 0 0 0 0]
end

%% Apply angle offsets to all joints
L_theta = L_theta + angle_offset';

%% -------------------------------------------- Numeric forward kinematics
fprintf('Numeric forward kinematics\n');

[T0_i, Tj_i] = get_T0i(L_theta, L_a, L_d, L_alpha);
for i = 1:n
    joint_pos{i} = T0_i{i}(1:3, 4); % Joint positions
end
L_pos_sol = round(joint_pos{end}, 6);

end
