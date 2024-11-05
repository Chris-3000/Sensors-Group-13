clf;

numImages = 4;
prefix = "lampjerk";
extension = '.jpg';

filenames = strings(1, numImages);
for i = 1:numImages
    filenames(i) = sprintf('%s%d%s', prefix, i, extension);
end

[imagePoints, boardSize] = detectCheckerboardPoints(filenames);
disp(size(imagePoints));

squareSize = 8; % millimetres
worldPoints = generateCheckerboardPoints(boardSize, squareSize);
disp(boardSize)

cameraParams = load('cameraParams.mat');
cameraParams = cameraParams.cameraParams;

Tc2p = zeros(4,4,numImages);
camExtrinsics(numImages,1) = rigidtform3d;
for index = 1:numImages
    I = imread(filenames(index));
    imageSize = [size(I, 1), size(I, 2)];
    
    % cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
    %                                         'WorldUnits', 'millimetres', ...
    %                                         'ImageSize',imageSize);
    [rotationMatrix, translationVector] = extrinsics(imagePoints(:,:,index), worldPoints, cameraParams);
    % camExtrinsics(index) = estimateExtrinsics(imagePoints(:,:,index), worldPoints, cameraParams)
    camExtrinsics(index) = [rotationMatrix, 0.001*translationVector'; zeros(1,3),1];
    
    disp('Rotation matrix:');
    disp(rotationMatrix);
    disp('Translation vector:');
    disp(translationVector);
    Tc2p(:,:,index) = [rotationMatrix, 0.001*translationVector'; zeros(1,3),1];
end
dobot = DobotMagician;

Tb2e = zeros(4,4,4);
Tb2e(:,:,1) = dobot.model.fkine([0, 0, 0.5*pi, 0, 0]).T;
Tb2e(:,:,2) = dobot.model.fkine([0, 0, 0.75*pi, 0, 0]).T;
Tb2e(:,:,3) = dobot.model.fkine([-0.25*pi, 0, 0.5*pi, 0, 0]).T;
Tb2e(:,:,4) = dobot.model.fkine([-0.25*pi, 0, 0.5*pi, 0, 0]).T;
Te2b = zeros(4,4,4);
for i = 1:4
    Te2b(:,:,i) = inv(Tb2e(:,:,i));
end

A = zeros(4,4,numImages-1);
B = zeros(4,4,numImages-1);
for i = 1:numImages-1
    A(:,:,i) = Tb2e(:,:,i+1) * inv(Tb2e(:,:,i));
    % A(:,:,2) = Tb2e(:,:,3) * inv(Tb2e(:,:,2));
    % A(:,:,3) = Tb2e(:,:,4) * inv(Tb2e(:,:,3));
    B(:,:,i) = Tc2p(:,:,i+1) * inv(Tc2p(:,:,i));
    % B(:,:,2) = Tc2p(:,:,3) * inv(Tc2p(:,:,2));
    % B(:,:,3) = Tc2p(:,:,4) * inv(Tc2p(:,:,3));
end
% X = AXXB_Solver(A, B);
% X = Tb2e(:,:,1) * trotx(pi) * inv(Tb2e(:,:,1))* X;
% X = transl(0.1, -0.9, 0.25) * trotx(-pi/2);
% X = helperEstimateHandEyeTransform(camExtrinsics,Te2b,'eye-to-hand');
% disp(X);

% dobot = DobotMagician;
% dobot.model.animate([0,0,pi/2,0,0]);

% camSim = CentralCamera('focal', mean(cameraParams.FocalLength), ...
%     'resolution', [1920, 1080], ...
%     'centre', [960, 540], ...
%     'name', 'mycam');
% camSim.T = X;
% for i = 1:numImages
%     disp(inv(Tb2e(:,:,i)) * X * Tc2p(:,:,i));
% end
% hold on
% camSim.plot_camera('scale', 0.1);
% axis([-1,1,-1,1,0,1]);

% Display x, y and z axes on checkerboard
axesPoints = [0, 0, 0; squareSize, 0, 0; 0, squareSize, 0; 0, 0, squareSize];
projectedPoints = worldToImage(cameraParams, rotationMatrix, translationVector, axesPoints);

imshow(I);
hold on;
axis on;
origin = projectedPoints(1,:);
xAxis = projectedPoints(2,:) - origin;
yAxis = projectedPoints(3,:) - origin;
zAxis = projectedPoints(4,:) - origin;
quiver(origin(1), ...
    origin(2), ...
    xAxis(1), ...
    xAxis(2), ...
    'LineWidth',2, ...
    'Color','red', ...
    'MaxHeadSize',10);
quiver(origin(1), ...
    origin(2), ...
    yAxis(1), ...
    yAxis(2), ...
    'LineWidth',2, ...
    'Color','green', ...
    'MaxHeadSize',10);
quiver(origin(1), ...
    origin(2), ...
    zAxis(1), ...
    zAxis(2), ...
    'LineWidth',2, ...
    'Color','blue', ...
    'MaxHeadSize',10);


