%% Reset plot and ROS connection
clear all;
clc;
clf;
close all;
rosshutdown;

%% Start Dobot Magician Node
rosinit('192.168.27.1');

%% Initialize Real Dobot and ROS Node
dobot = DobotROS();

%% Simulate Dobot Magician in MATLAB
dobotSim = DobotMagician;
dobotSim.useTool = false;
hold on;
axis([-3, 3, -3, 3, 0, 3]);

%% Initialize Camera and Calibration Parameters
cam = webcam();
cameraParams = load('cameraParams.mat');
cameraParams = cameraParams.cameraParams;
worldPoints = generateCheckerboardPoints([5,8], 8);

%% Simulation Camera Setup
camSim = CentralCamera('focal', mean(cameraParams.FocalLength), ...
    'resolution', [1920, 1080], ...
    'centre', [960, 540], ...
    'name', 'Logitech BRIO');

%% Hand-Eye Calibration Parameters
poses = 6;
Tc2p = zeros(4,4,poses);
Tb2e = zeros(4,4,poses);

% Camera measurements for multiple poses
for i = 1:poses
    % Move Dobot to the specified joint configuration
    q = [-0.8 + i * 0.2, 0.4, 0.4, 0];
    dobot.PublishTargetJoint(q);
    pause(2); % Ensure the robot has time to reach the position

    % Record end-effector transform relative to base
    Tb2e(:,:,i) = dobotSim.model.fkine(q).T;

    % Capture and process the image for checkerboard detection
    success = false;
    attempts = 0;
    while ~success && attempts < 5
        frame = snapshot(cam);
        [points, boardSize] = detectCheckerboardPoints(frame);
        if size(points, 1) == 28
            [rotationMatrix, translationVector] = extrinsics(points, worldPoints, cameraParams);
            Tc2p(:,:,i) = [rotationMatrix, 0.001 * translationVector'; zeros(1,3), 1];
            success = true;
            disp("Image successfully captured and processed.");
        else
            attempts = attempts + 1;
            disp("Not enough points detected. Retrying...");
        end
    end
    if ~success
        warning("Unable to detect checkerboard in this position.");
        break;
    end
end

%% Calculate Transformations A and B
A = zeros(4,4,poses-1);
B = zeros(4,4,poses-1);
for i = 1:poses-1
    A(:,:,i) = Tb2e(:,:,i+1) * inv(Tb2e(:,:,i));
    B(:,:,i) = Tc2p(:,:,i+1) * inv(Tc2p(:,:,i));
end

% Compute hand-eye calibration transformation X
estimatedTc = AXXB_Solver(A, B);

%% Display Results
disp('Estimated Camera Pose (Tc):');
disp(estimatedTc);

% Plot the estimated camera pose in the simulation
camSim.T = estimatedTc;
camSim.plot_camera('scale', 0.1);