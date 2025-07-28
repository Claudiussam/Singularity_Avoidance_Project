function D = forwardTransfer(a, alpha, d, theta)
% FORWARDTRANSFER Compute DH transformation matrix for a 5 DOF robot
% Inputs: DH parameters (a, alpha, d, theta) for a single joint
% Output: Homogeneous transformation matrix D

% Rotation about z-axis by theta
A = [cos(theta) -sin(theta)   0    0;
      sin(theta)  cos(theta)   0    0;
      0           0            1    0;
      0           0            0    1];
A = vpa(A, 3); % Symbolic precision control

% Translation along x-axis by 'a' and z-axis by 'd'
B = [1  0   0   a;
      0  1   0   0;
      0  0   1   d;
      0  0   0   1];
B = vpa(B, 3);

% Rotation about x-axis by alpha
C = [1  0           0           0;
      0  cos(alpha) -sin(alpha)  0;
      0  sin(alpha)  cos(alpha)  0;
      0  0           0           1];
C = vpa(C, 3);

% Combined transformation matrix
D = A * B * C;

end