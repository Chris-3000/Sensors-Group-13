function cameraParams = webcamSequence(webcamName)

    cam = webcam(webcamName); % specify used webcam
    boardSize = [5, 8];
    squareSize = 10; % millimetres
    nPictures = 3;
    nPoints = (boardSize(1) - 1) * (boardSize(2) - 1);
    imagePoints = zeros(nPoints,2,nPictures);
    
    preview(cam);
    
    for i = 1:nPictures
        disp('Press enter to capture a picture. Ensure that the test pattern is in frame.');
        pause();
        img = snapshot(cam);
        detectedPoints = detectCheckerboardPoints(img);
        disp(size(detectedPoints));
        imagePoints(:,:,i) = detectedPoints(1:nPoints,:);
    end
    
    worldPoints = generateCheckerboardPoints(boardSize, squareSize);
    imageSize = [size(img, 1), size(img, 2)];
    cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
                                            'WorldUnits', 'millimetres', ...
                                            'ImageSize',imageSize);
    
    % save("cameraParams.mat", "cameraParams");
    
    clear('cam');

end
