n = 10;
A = zeros(4, 4, n);
B = zeros(4, 4, n);
X = trotx(rand(1)) * troty(rand(1)) * trotz(rand(1)) * transl(rand(1), rand(1), rand(1));
for i = 1:n
    A(:, :, i) = trotx(rand(1)) * troty(rand(1)) * trotz(rand(1)) * transl(rand(1), rand(1), rand(1));
    B(:, :, i) = inv(X) * A(:, :, i) * X;
end
disp(X)
disp(AXXB_Solver(A, B));