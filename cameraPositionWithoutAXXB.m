% Reset everything
clear all;
clc;
clf;
close all;
rosshutdown;

% Start Dobot Magician Node
rosinit('192.168.27.1');

% Start Dobot ROS
dobot = DobotROS();

% Control the real dobot and move it to the expected default position 
q = [0,0,0,0];
dobot.PublishTargetJoint(q);

% Plot dobot in simulation to the same position it is in real life
dobotSim = DobotMagician();
dobotSim.useTool = false;
qSim = [0,0,pi/2,pi/2];
dobotSim.model.animate(qSim);
drawnow

hold on
% axis equal
Tb2e = dobotSim.model.fkine(qSim).T *trotz(pi);
Te2p = transl(0.013,0.03,0.044+0.015) * troty(pi/2) * trotz(-pi/2);
disp(Tb2e)
drawnow

%% Initiate camera
cam = webcam('Logitech BRIO');
Tc2p = zeros(4,4);
worldPoints = generateCheckerboardPoints([5,8], 10);
intrinsics = load('cameraParams.mat');
cameraParams = intrinsics.cameraParams;

%% Take picture until the pattern is properly detected
while true
    frame = snapshot(cam);
    [points, boardSize] = detectCheckerboardPoints(frame);
    if size(points, 1) == 28
        [rotationMatrix, translationVector] = extrinsics(points, worldPoints, cameraParams);
        Tc2p = [rotationMatrix, 0.001*translationVector'; zeros(1,3), 1];
        disp("Image taken");
        break;
    else
        disp("Not enough points detected");
    end
end

%% Plot camera in simulation where it is estimated to be
camSim = CentralCamera('focal', mean(cameraParams.FocalLength), ...
    'resolution', [2560, 1440], ...
    'centre', [1280, 720], ...
    'name', 'Logitech BRIO');
camSim.T = Tb2e * Te2p * inv(Tc2p);
camSim.plot_camera('scale', 0.1);

