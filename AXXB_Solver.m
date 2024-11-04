% % Credit: https://github.com/LarryDong/HandEye-Tsai/blob/main/tsai.m
% 
% function [X] = AXXB_Solver(A, B)
%     % A is an array of 4x4 pose matrices, B is the same
%     % X is a 4x4 pose matrix that ideally satisfies the equation AX=XB for 
%     % all A and B
% 
%     %% Creation of important local variables
%     n = size(A, 3);
%     R_A = A(1:3, 1:3, :);
%     t_A = A(1:3, 4, :);
%     R_B = B(1:3, 1:3, :);
%     t_B = B(1:3, 4, :);
%     R_X = eye(3);
% 
%     %% Calculating best rotation R_X
%     % Eq. 12: skew(a + b) * x = a - b 
%     % S * x = v => x = S \ v
%     S = zeros(3*n,3);
%     v = zeros(3*n,1);
% 
%     %% Calculate best rotation R
%     for i = 1:n
%         % log(R) = theta * skew(r)
%         % r is the unit axis of rotation vector
%         % theta is the angle of of rotation
%         A1 = logm(R_A(:, :, i));
%         B1 = logm(R_B(:, :, i));
% 
%         % a and b are the unit axes of rotation of A and B respectively
%         % a = [A1(3,2) A1(1,3) A1(2,1)]'; a = a/norm(a); 
%         % b = [B1(3,2) B1(1,3) B1(2,1)]'; b = b/norm(b); 
%         a = [A1(3,2) A1(1,3) A1(2,1)]'; a = a/norm(a); 
%         b = [B1(3,2) B1(1,3) B1(2,1)]'; b = b/norm(b); 
% 
%         S(3*i-2:3*i,:) = skew(a+b);   
%         v(3*i-2:3*i,:) = a-b;  
%     end
% 
%     x = S\v; % Eq. 12 
%     theta = 2*atan(norm(x)); % Eq. 13
%     eps = 0.0000000000001; % avoid divide-by-zero. Credit to LarryDong
%     x = x/(norm(x)+eps); % normalise x
%     R_X = (eye(3)*cos(theta) + sin(theta)*skew(x) + (1-cos(theta))*x*x')'; % Rodrigues formula
% 
%     %% Calculating best translation t_X
%     % R_A * t_X + t_A = R_X * t_B + t_X
%     % (I - R_A) * t_X = t_A - R_X * t_B
%     % t_X = (I - R_A) \ (t_A - R_X * t_B) = C \ d
%     C = zeros(3*n,3);
%     d = zeros(3*n,1);
% 
%     for i = 1:n
%         % C = I - R_A
%         C(3*i-2:3*i,:) = eye(3) - R_A(:, :, i);
% 
%         % D = t_A - R_X * t_B
%         d(3*i-2:3*i,:) = t_A(:, :, i) - R_X * t_B(:, :, i);
%     end
% 
%     t_X = C \ d;
% 
%     %% Final answer
%     X = [R_X, t_X; zeros(1,3), 1];
% end

function X = AXXB_Solver(A, B)
    % Ensure we have at least 3 poses to solve accurately
    numPoses = size(A, 3);
    if numPoses < 3
        error('AXXB_Solver requires at least 3 pose pairs for a stable solution.');
    end

    % Step 1: Calculate the rotation part of X
    RA = zeros(3, 3 * numPoses);
    RB = zeros(3, 3 * numPoses);
    for i = 1:numPoses
        RA(:, (i-1)*3 + (1:3)) = A(1:3, 1:3, i);
        RB(:, (i-1)*3 + (1:3)) = B(1:3, 1:3, i);
    end
    
    % Compute the rotation matrix of X using SVD
    H = RA * RB';
    [U, ~, V] = svd(H);
    R_X = U * V';
    
    % Ensure we have a proper rotation matrix (det(R_X) should be 1)
    if det(R_X) < 0
        R_X = U * diag([1, 1, -1]) * V';
    end
    
    % Step 2: Calculate the translation part of X
    % Set up a system of equations for each pose to solve for translation
    tA = zeros(3, numPoses);
    tB = zeros(3, numPoses);
    for i = 1:numPoses
        tA(:, i) = A(1:3, 4, i);
        tB(:, i) = R_X * B(1:3, 4, i);
    end

    % Solve for the translation vector by averaging across poses
    t_X = mean(tA - tB, 2);

    % Combine rotation and translation into homogeneous transform
    X = eye(4);
    X(1:3, 1:3) = R_X;
    X(1:3, 4) = t_X;
end
