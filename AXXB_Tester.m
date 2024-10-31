n = 2;
A = zeros(4, 4, n);
B = zeros(4, 4, n);
Ts = zeros(4, 4, 3);
Td = zeros(4, 4, 3);
X = trotx(rand(1)) * troty(rand(1)) * trotz(rand(1)) * transl(rand(1), rand(1), rand(1));
for i = 1:n
    A(:, :, i) = trotx(rand(1)) * troty(rand(1)) * trotz(rand(1)) * transl(rand(1), rand(1), rand(1));
    B(:, :, i) = inv(X) * A(:, :, i) * X;
end
B(3,4,1) = B(3,4,1) + 0.00001;
% Ts = 
% A(:,:,1) = transl(0,0,0.025) * inv(transl(0,0,0.02));
% A(:,:,2) = transl(0,0,0.035) * inv(transl(0,0,0.025));
% B(:,:,1) = transl(0.05, -0.025, 0.4) * inv(transl(0.05, -0.02, 0.4));
% B(:,:,2) = transl(0.05, -0.035, 0.4) * inv(transl(0.05, -0.025, 0.4));
% disp(A)
% disp(B)
disp(X);
disp(AXXB_Solver(A, B));