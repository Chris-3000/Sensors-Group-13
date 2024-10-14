% Credit: https://github.com/LarryDong/HandEye-Tsai/blob/main/tsai.m

function [X] = AXXB_Solver(A, B)
    % A is an array of 4x4 pose matrices, B is the same
    % X is a 4x4 pose matrix that ideally satisfies the equation AX=XB for 
    % all A and B

    %% Creation of important local variables
    n = size(A, 3);
    R_A = A(1:3, 1:3, :);
    t_A = A(1:3, 4, :);
    R_B = B(1:3, 1:3, :);
    t_B = B(1:3, 4, :);
    R_X = eye(3);

    %% Calculating best rotation R_X
    % Eq. 12: skew(a + b) * x = a - b 
    % S * x = v => x = S \ v
    S = zeros(3*n,3);
    v = zeros(3*n,1);
    
    %% Calculate best rotation R
    for i = 1:n
        % log(R) = theta * skew(r)
        % r is the unit axis of rotation vector
        % theta is the angle of of rotation
        A1 = logm(R_A(:, :, i));
        B1 = logm(R_B(:, :, i));

        % a and b are the unit axes of rotation of A and B respectively
        a = [A1(3,2) A1(1,3) A1(2,1)]'; a = a/norm(a); 
        b = [B1(3,2) B1(1,3) B1(2,1)]'; b = b/norm(b); 

        S(3*i-2:3*i,:) = skew(a+b);   
        v(3*i-2:3*i,:) = a-b;  
    end
    
    x = S\v; % Eq. 12 
    theta = 2*atan(norm(x)); % Eq. 13
    eps = 0.0000000000001; % avoid divide-by-zero. Credit to LarryDong
    x = x/(norm(x)+eps); % normalise x
    R_X = (eye(3)*cos(theta) + sin(theta)*skew(x) + (1-cos(theta))*x*x')'; % Rodrigues formula

    %% Calculating best translation t_X
    % R_A * t_X + t_A = R_X * t_B + t_X
    % (I - R_A) * t_X = t_A - R_X * t_B
    % t_X = (I - R_A) \ (t_A - R_X * t_B) = C \ d
    C = zeros(3*n,3);
    d = zeros(3*n,1);

    for i = 1:n
        % C = I - R_A
        C(3*i-2:3*i,:) = eye(3) - R_A(:, :, i);

        % D = t_A - R_X * t_B
        d(3*i-2:3*i,:) = t_A(:, :, i) - R_X * t_B(:, :, i);
    end

    t_X = C \ d;

    %% Final answer
    X = [R_X, t_X; zeros(1,3), 1];
end

