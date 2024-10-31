function q = ikineDobot(x, y, z)
    q = zeros(1,4);
    q(1) = atan2(y,x);

    A = 0.135;
    B = 0.147;
    S = sqrt(x^2+y^2) - 0.07;
    C = 0.1392 - z;
    D = sqrt((2*S*B)^2+(2*C*B)^2);
    theta = atan2(S,C);
    gamma = 2*pi - theta - acos((A^2-S^2-B^2-C^2)/D);

    q(2) = asin((S-B*sin(gamma))/A);
    q(3) = gamma - q(2);
    % q(4) = pi - q(3) - q(2);
end