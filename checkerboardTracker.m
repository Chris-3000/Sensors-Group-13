% cam = webcam('Logitech BRIO');
cam = webcam();

% pause(10);

while true
    frame = snapshot(cam);
    [points, boardSize] = detectCheckerboardPoints(frame);
    if size(points, 1) == 28
        pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
        initialize(pointTracker, points, frame);
        break;
    else
        disp("Not enough points detected");
    end
end


cameraParams = load('cameraParams.mat');
cameraParams = cameraParams.cameraParams;
worldPoints = generateCheckerboardPoints([5,8], 8);
T = ones(4);
Ts = zeros(4,4,3);
Td = zeros(4,4,3);
figure;
tic

% TRand1 = trotx(rand(1)) * troty(rand(1)) * trotz(rand(1)) * transl(rand(1), rand(1), rand(1));
% TRand2 = TRand1 * trotx(rand(1) * 0.01);
% TRand3 = TRand1 * trotx(rand(1)*0.01);
while ishandle(gca)  % Continue until the figure is closed
    % Capture the next frame
    frame = snapshot(cam);

    % Track the points in the new frame
    [trackedPoints, validity] = step(pointTracker, frame);

    % Keep only valid points
    validPoints = trackedPoints(validity, :);

    % Display the current frame with tracked points
    imshow(frame); hold on;
    plot(validPoints(:, 1), validPoints(:, 2), 'ro', 'MarkerSize', 20, 'LineWidth',5);
    hold off;
    % cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
    %                                         'WorldUnits', 'millimetres', ...
    %                                         'ImageSize',imageSize);
    % disp()
    % worldPoints = generateCheckerboardPoints([5,8], 10);
    % [rotationMatrix, translationVector] = extrinsics(validPoints, worldPoints, cameraParams);
    % disp([rotationMatrix, translationVector'; zeros(1,3), 1]);
    % disp(rotationMatrix);
    % disp(translationVector);

    drawnow;  % Update the display

    % Check if points were lost, and reinitialize if necessary
    if sum(validity) < 28  % Threshold to re-detect points if most are lost
        disp('Reinitializing points...');
        [points, boardSize] = detectCheckerboardPoints(frame);
        if size(points, 1) == 28
            setPoints(pointTracker, points);
        end
    else
        [rotationMatrix, translationVector] = extrinsics(validPoints, worldPoints, cameraParams);
        % translationVector(3) = translationVector(3) + 120;
        T = [rotationMatrix, 0.001*translationVector'; zeros(1,3), 1];
        disp(T);
    end
    % if toc >= 5 && toc < 6
    %     disp('taking pic 1');
    %     Ts(:,:,1) = T;
    %     % Td(:,:,1) = T * TRand;
    %     Td(:,:,1) = transl(0,0,0.02) * trotx(rand(1)*0.001);
    % end
    % if toc >= 10 && toc < 11
    %     disp('taking pic 2');
    %     Ts(:,:,2) = T;
    %     % Td(:,:,2) = T * TRand;
    %     Td(:,:,2) = transl(0,0,0.025) * trotx(rand(1)*0.001);
    % end
    % if toc >= 15 && toc < 16
    %     disp('taking pic 3');
    %     Ts(:,:,3) = T;
    %     % Td(:,:,3) = T * TRand;
    %     Td(:,:,3) = transl(0,0,0.028) * trotx(rand(1)*0.001);
    % end
    % if toc >= 16
    %     break;
    % end
end

% disp(Ts);
% disp(Td);
% A = zeros(4,4,2);
% B = zeros(4,4,2);
% A(:,:,1) = Td(:,:,2) * inv(Td(:,:,1));
% A(:,:,2) = Td(:,:,3) * inv(Td(:,:,2));
% B(:,:,1) = Ts(:,:,2) * inv(Ts(:,:,1));
% B(:,:,2) = Ts(:,:,3) * inv(Ts(:,:,2));
% disp(A)
% disp(B)
% X = AXXB_Solver(A, B);
% 
% disp(X);

clear cam;
release(pointTracker);