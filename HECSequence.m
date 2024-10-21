cam = webcam(); % specify used webcam
boardSize = [5, 8];
squareSize = 10; % millimetres
nPictures = 3;
nPoints = (boardSize(1) - 1) * (boardSize(2) - 1);
imagePoints = zeros(nPoints,2,nPictures);

preview(cam);

for i = 1:nPictures
    % move end effector to desired location
    pause(3);
    img = snapshot(cam);
    detectedPoints = detectCheckerboardPoints(img);
    % disp(size(detectedPoints));
    imagePoints(:,:,i) = detectedPoints(1:nPoints,:);
end

worldPoints = generateCheckerboardPoints(boardSize, squareSize);
imageSize = [size(img, 1), size(img, 2)];
% cameraParams = load('cameraParams.mat');

for i = 1:nPictures
    disp(imagePoints(:,:,i));
    [rotationMatrix, translationVector] = extrinsics(imagePoints(:,:,i), worldPoints, cameraParams);
    T = [rotationMatrix, translationVector' * 0.001; zeros(1,3), 1];
    disp(T);
end

clear('cam');