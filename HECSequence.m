%% Ros clearing
% clear all;
% clc;
% close all;
rosshutdown;

%% camera calibration
cameraParams = webcamSequence("Logitech BRIO");
disp('Press enter when the camera is in position');
pause();


%% regular params
cam = webcam("Logitech BRIO"); % specify used webcam
boardSize = [5, 8];
squareSize = 10; % millimetres
nPictures = 3;
nPoints = (boardSize(1) - 1) * (boardSize(2) - 1);
imagePoints = zeros(nPoints,2,nPictures);
T_B2E = zeros(4,4,3);
T_C2P = zeros(4,4,3);
A = zeros(4,4,2);
B = zeros(4,4,2);

preview(cam);

dobotSim = DobotMagician();



%% Ros starting
rosinit('192.168.27.1');

%% get the dobot in
dobot = DobotROS();


%% Shenanigans
for i = 1:nPictures
    joint_target = [-0.2 * i,0.4,0.3,0.0];
    q = zeros(1,5);
    q(1) = joint_target(1);
    q(2) = joint_target(2);
    q(3) = joint_target(3);
    q(4) = pi - q(3) - q(2);
    q(5) = joint_target(4);
    T_B2E(:,:,i) = dobotSim.model.fkine(q);
    dobotSim.model.animate(q);
    % end_effector_position = [0.05 + i * 0.05,-0.1,0.05];
    % end_effector_rotation = [0,0,0];
   
    % dobot.PublishEndEffectorPose(end_effector_position,end_effector_rotation);
    % T_B2E(:,:,i) = transl(end_effector_position);
    % move end effector to desired location
    pause(3);
    img = snapshot(cam);
    detectedPoints = detectCheckerboardPoints(img);
    % disp(size(detectedPoints));
    imagePoints(:,:,i) = detectedPoints(1:nPoints,:);
end

%% Post shenaningans

worldPoints = generateCheckerboardPoints(boardSize, squareSize);
imageSize = [size(img, 1), size(img, 2)];
% cameraParams = load('cameraParams.mat');

%% post post

for i = 1:nPictures
    disp(imagePoints(:,:,i));
    [rotationMatrix, translationVector] = extrinsics(imagePoints(:,:,i), worldPoints, cameraParams);
    T = [rotationMatrix, translationVector' * 0.001; zeros(1,3), 1];
    T_C2P(:,:,i) = T;
end

A(:,:,1) = T_B2E(:,:,2) * inv(T_B2E(:,:,1));
A(:,:,2) = T_B2E(:,:,3) * inv(T_B2E(:,:,2));
B(:,:,1) = T_C2P(:,:,2) * inv(T_C2P(:,:,1));
B(:,:,2) = T_C2P(:,:,3) * inv(T_C2P(:,:,2));
X = AXXB_Solver(A, B);
disp(X)
hold on;
plot(X(1:3,4)', "ro");
%% fin
clear('cam');
% clear('dobot');