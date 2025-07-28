function theta4 = findTh4(theta)
% FINDTH4 Calculate theta4 for a 5 DOF robot
%   theta: Vector of joint angles [theta1, theta2, theta3, theta4, theta5]

% Compute theta4 using the same relationship as before
theta4 = pi/2 - sum(theta(2:3));
end