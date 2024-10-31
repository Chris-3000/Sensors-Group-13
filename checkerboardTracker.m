% cam = webcam('Logitech BRIO');
cam = webcam;

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

% if ~isempty(points)
%     % Initialize the tracker with detected points
%     pointTracker = vision.PointTracker('MaxBidirectionalError', 2);
%     initialize(pointTracker, points, frame);
% else
%     error('No points detected in the initial frame.');
% end

figure;
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

    drawnow;  % Update the display

    % Check if points were lost, and reinitialize if necessary
    if sum(validity) < 28  % Threshold to re-detect points if most are lost
        disp('Reinitializing points...');
        [points, boardSize] = detectCheckerboardPoints(frame);
        if size(points, 1) == 28
            setPoints(pointTracker, points);
        end
    end
end

clear cam;
release(pointTracker);