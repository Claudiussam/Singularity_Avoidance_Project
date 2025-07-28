function [T0_i, Tj_i] = get_T0i(theta, a, d, alpha)
% GET_T0I Compute transformation matrices for a 5 DOF robot
%   Returns:
%   - T0_i: Cell array of transformations from base to joint i
%   - Tj_i: Cell array of individual joint transformations

n = length(a); % Number of joints (5 for 5 DOF)

% Initialize cells with n+1 to accommodate T0_j{1} to T0_j{n+1}
T0_i = cell(1, n);
Tj_i = cell(1, n);
T0_j = cell(1, n+1); % Corrected initialization for 5 DOF (n+1 cells)

T0_j{1} = eye(4); % Base frame transformation

for i = 1:n
    % Compute transformation for joint i
    Tj_i{i} = forwardTransfer(a(i), alpha(i), d(i), theta(i));
    
    % Compute cumulative transformation from base to joint i
    T0_i{i} = T0_j{i} * Tj_i{i};
    
    % Store result for next iteration
    T0_j{i+1} = T0_i{i};
end

% Optional: Assign variables to base workspace for debugging
assignin('base', 'Tj_i', Tj_i);
assignin('base', 'T0_j', T0_j);

end