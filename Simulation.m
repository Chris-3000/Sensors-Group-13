clf

dobot = DobotMagician;
dobot.useTool = false;
q = dobot.model.ikine(transl(0.2,0.2,0.2), 'mask', [1,1,1,0,0,0]);
q(4) = pi-q(3)-q(2);
dobot.model.animate(q)

intrinsics = load('cameraParams.mat');
intrinsics = intrinsics.cameraParams;
cam = CentralCamera('focal', mean(intrinsics.FocalLength), ...
    'resolution', [1920, 1080], ...
    'centre', [960, 540], ...
    'name', 'Logitech BRIO');
cam.T = transl(1,0,0) * trotx(pi/2) * trotz(pi) * troty(pi/2);


axis([-2,2,-2,2,0,2]);
% axis equal
hold on

function corners = patternCorners(q, dobot)
    T = dobot.model.fkine(q).T * transl(-0.05, 0, 0);
    ptl = T * transl(0, 0.04, 0.025);
    ptr = T * transl(0, -0.04, 0.025);
    pbl = T * transl(0, 0.04, -0.025);
    pbr = T * transl(0, -0.04, -0.025);
    corners = [ptl(1:3,4) ptr(1:3,4) pbl(1:3,4) pbr(1:3,4)];
end

cam.plot_camera('scale', 0.1);


steps = 800;
% qMatrix = zeros(steps,5);

% xMatrix = zeros(3,steps);
% m = zeros(1,steps);

% x1 = [0.3; -0.1; 0.1];
% x2 = [0.2; 0.2; 0.2];
% s = linspace(0,1,steps);
% for i = 1:steps
%     xMatrix(:,i) = x1 * (1 - s(i)) + x2 * s(i);
% end

q0 = ikineDobot(0.21, 0.21, 0.21);
cornerGoal = patternCorners(ikineDobot(0.2, 0.2, 0.2), dobot);
uvStar = cam.plot(cornerGoal);
q = q0;
lambda = 0.1;

% pause();
for i = 1:steps-1
    uv = cam.plot(patternCorners(q, dobot));
    e = uvStar - uv;
    e = e(:);
    Jcam = cam.visjac_p(uv, 0.9);
    v = lambda * pinv(Jcam) * e;
    J = dobot.model.jacobe(q);
    % J = J(1:3,:);
    % J(:,4:5) = zeros(3,2);
    % qDot = J'*inv(J*J')*v(1:3);
    qDot = J' * pinv(J * J' + 0.003 * eye(6))*v;
    qDotMax = pi/2;
    ind=find(qDot>qDotMax);
    if ~isempty(ind)
        qDot(ind)=qDotMax;
    end
    ind=find(qDot<-qDotMax);
    if ~isempty(ind)
        qDot(ind)=-qDotMax;
    end
    q = q + qDot' * 0.1; % replace 0.1 with fps
    % q(2) = q(2) - 0.1;
    % q(3) = q(3) - 0.1;
    q(4) = pi - q(3) - q(2);
    q(5) = 0;


    % t = dobot.model.fkine(qMatrix(i,:)).t;
    % xDot = xMatrix(:,i+1) - t;
    % J = dobot.model.jacob0(qMatrix(i,:));
    % J = J(1:3,:);
    % J(1:3,4:5) = zeros(3,2);
    % m(:,i) = sqrt(det(J*J'));   % Measure of Manipulability
    % if m(:,i) > epsilon
    %     lambda = 0;
    % else
    %     lambda = (1 - (m(:,i)/epsilon)^2) * lambdaMax;
    % end
    % qDot = J'*inv(J*J' + lambda * eye(3))*xDot;
    % qMatrix(i+1,:)= qMatrix(i,:) + (qDot)';
    % qMatrix(i+1,4) = pi - qMatrix(i+1,3) - qMatrix(i+1,2);
    dobot.model.animate(q);
    cam.clf();

    %% Corners of the pattern in camera view
    % T = dobot.model.fkine(qMatrix(i+1,:)).T * transl(-0.05, 0, 0);
    % ptl = T * transl(0, 0.04, 0.025);
    % ptr = T * transl(0, -0.04, 0.025);
    % pbl = T * transl(0, 0.04, -0.025);
    % pbr = T * transl(0, -0.04, -0.025);
    % cam.plot(ptl(1:3,4)); 
    % cam.plot(ptr(1:3,4)); 
    % cam.plot(pbl(1:3,4)); 
    % cam.plot(pbr(1:3,4)); 
    cam.plot(patternCorners(q, dobot));
    cam.plot(cornerGoal, '*r');

    cam.hold(true);
    drawnow
    pause(0.1);
end


% 
% % P1 = dobot.model.fkine(q).t;
% % disp(P1)
% P1 = [0.2; 0.2; 0.2];
% plot_sphere(P1, 0.05, 'r');
% cam.clf();
% cam.plot(P1); 
% cam.hold(true);