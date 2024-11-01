%% Reset plot
clf;

%% Initiate Dobot Magician
dobot = DobotMagician;
dobot.useTool = false;
q0 = ikineDobot(0.2, 0, 0.2);
% q0 = dobot.model.ikcon(trotx(rand(1)) * troty(rand(1)) * trotz(rand(1)) * transl(0.1*rand(1), 0.1*rand(1), 0.1*rand(1)));
disp(q0)
dobot.model.animate(q0);
hold on
axis([-2, 2, -2, 2, 0, 2]);

%% Initiate camera
intrinsics = load('cameraParams.mat');
intrinsics = intrinsics.cameraParams;
cam = CentralCamera('focal', mean(intrinsics.FocalLength), ...
    'resolution', [1920, 1080], ...
    'centre', [960, 540], ...
    'name', 'Logitech BRIO');
Tc = trotz(0) * transl(2,0,0.3) * trotz(pi/2) * trotx(-pi/2);
% Tc = trotx(rand(1)) * troty(rand(1)) * trotz(rand(1)) * transl(rand(1), rand(1), rand(1));

cam.T = Tc;
cam.plot_camera('scale', 0.1);

%% Hand-Eye Calibration
% Initiate camera measurement parameters
poses = 6;
Tc2p = zeros(4,4,poses);
Tb2e = zeros(4,4,poses);

% Camera measurements
for i = 1:poses
    % Record end effector transform relative to base
    q = ikineDobot(0.15, 0.2*(2*rand(1)-1), 0.2*rand(1));
    % q = dobot.model.ikcon(trotx(rand(1)) * troty(rand(1)) * trotz(rand(1)) * transl(0.1*rand(1), 0.1*rand(1), 0.1*rand(1)));
    Tb2e(:,:,i) = dobot.model.fkine(q).T;

    % Record pattern transform relative to camera
    Tp = Tb2e(:,:,i) * troty(-pi/2) * trotz(pi/2);
    Tc2p(:,:,i) = Tc \ Tp;

    % Animate the robot in position
    dobot.model.animate(q);
    drawnow
end

% Estimate transform of camera relative to base
A = zeros(4,4,poses-1);
B = zeros(4,4,poses-1);
for i = 1:poses-1
    A(:,:,i) = Tb2e(:,:,i+1) * inv(Tb2e(:,:,i));
    B(:,:,i) = Tc2p(:,:,i+1) * inv(Tc2p(:,:,i));
end
estimatedTc = AXXB_Solver(A,B);

% Display estimated versus real values
disp('Estimated Camera Pose: ')
disp(estimatedTc);
disp('Actual Camera Pose: ')
disp(Tc)

%% Visual Servoing

id = 1; % Note: may need to be changed if multiple joysticks present
joy = vrjoystick(id);
caps(joy) % display joystick information

hold on
% x = 0.1;
% y = -0.1;
% z = 1.8;
% Tc2p = transl(x, y, z);
% TDesired = estimatedTc * Tc2p * transl(0,0,0.1);
% tDesired = TDesired(1:3,4);
tDesired = [0.2, 0.2, 0.1];
targetPoint = plot3(tDesired(1), tDesired(2), tDesired(3), '*r', 'MarkerSize',20);
tCurrent = [0.3, -0.1, 0.2];
qCurrent = ikineDobot(tCurrent(1), tCurrent(2), tCurrent(3));
dobot.model.animate(qCurrent);
drawnow
pause();
lambda = 0.1;
steps = 500;
K = 0.01;
% qMatrix = jtraj(qCurrent, qDesired, steps);
for i = 1:steps
    [axes, buttons, povs] = read(joy);
    % 
    x = K * axes(1);
    y = K * axes(2);
    z = K * (buttons(8)-buttons(7));
    disp([x;y;z])
    % x = 0.03;
    % y = 0;
    % z = 0;

    % Tc2p = transl(x, y, z);
    % TDesired = estimatedTc * Tc2p * transl(0,0,0.1);
    % tDesired = TDesired(1:3,4);
    % tDesired = [0.2, 0.2, 0.1];
    tDesired = tDesired + [x; y; z];
    delete('targetPoint');
    targetPoint = plot3(tDesired(1), tDesired(2), tDesired(3), '*r', 'MarkerSize',20);
    % qDesired = ikineDobot(tDesired(1), tDesired(2), tDesired(3));
    qDesired = dobot.model.ikcon(transl(tDesired(1), tDesired(2), tDesired(3)));
    qDesired(4) = pi - qDesired(3) - qDesired(2);
    qCurrent = qCurrent + lambda * (qDesired - qCurrent);

% % TDesired = estimatedTc * Tc2p;
% % tDesired = TDesired(1:3,4);
% tDesired = [0.2, 0.2, 0.1];
% plot3(tDesired(1), tDesired(2), tDesired(3), '*r', 'MarkerSize',20);
% tCurrent = [0.3, -0.1, 0.2];
% qCurrent = ikineDobot(tCurrent(1), tCurrent(2), tCurrent(3));
% dobot.model.animate(qCurrent);
% drawnow
% pause();
% lambda = 0.01;
% steps = 800;
% % qMatrix = jtraj(qCurrent, qDesired, steps);
% for i = 1:steps
%     % TDesired = estimatedTc * Tc2p;
%     % tDesired = TDesired(1:3,4);
%     tDesired = [0.2, 0.2, 0.1];
%     qDesired = ikineDobot(tDesired(1), tDesired(2), tDesired(3));
%     qCurrent = qCurrent + lambda * (qDesired - qCurrent);
%     dobot.model.animate(qCurrent);
%     drawnow
% end



%% Visual Servoing - Refined Movement
tDesired = [0.2, 0.2, 0.1];
plot3(tDesired(1), tDesired(2), tDesired(3), '*r', 'MarkerSize', 20);
tCurrent = [-0.3, -0.1, 0.2];
qCurrent = ikineDobot(tCurrent(1), tCurrent(2), tCurrent(3));
dobot.model.animate(qCurrent);
drawnow;

lambda = 0.02;
steps = 500;

for i = 1:steps
    % Calculate the error in task space (position error)
    error = tDesired - tCurrent;

    % Move current joint configuration toward desired based on lambda
    delta_q = lambda * (ikineDobot(tDesired(1), tDesired(2), tDesired(3)) - qCurrent);
    qCurrent = qCurrent + delta_q;

    % Update current end-effector position based on qCurrent
    tCurrent = transl(dobot.model.fkine(qCurrent).T);

    % Animate the robot and update the display
    dobot.model.animate(qCurrent);
    drawnow;

    % Check convergence
    if norm(error) < 0.01
        disp('Target position reached');
        break;
    end
end


