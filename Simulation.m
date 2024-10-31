clf

dobot = DobotMagician;
dobot.useTool = false;
% q = dobot.model.ikine(transl(0.2,0.2,0.2), 'mask', [1,1,1,0,0,0]);
% q(4) = pi-q(3)-q(2);
% dobot.model.animate(q)

intrinsics = load('cameraParams.mat');
intrinsics = intrinsics.cameraParams;
cam = CentralCamera('focal', mean(intrinsics.FocalLength), ...
    'resolution', [1920, 1080], ...
    'centre', [960, 540], ...
    'name', 'Logitech BRIO');
% cam.T = transl(1,0,0) * trotx(pi/2) * trotz(pi) * troty(pi/2);
cam.T = transl(0,0,-2);


axis([-2,2,-2,2,0,2]);
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
% arrow = quiver3(0, 0, 0, 0, 0, 0);

steps = 800;

q0 = ikineDobot(0.2, -0.2, 0.2);
dobot.model.animate(q0);
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
    disp(v)
    % disp(v)
    % J = J(1:3,:);
    % J(:,4:5) = zeros(3,2);
    % disp(J * J');
    qDot = J' * inv(J * J' + 0.003 * eye(6)) * v;
    qDotMax = pi/2;
    ind=find(qDot>qDotMax);
    if ~isempty(ind)
        qDot(ind)=qDotMax;
    end
    ind=find(qDot<-qDotMax);
    if ~isempty(ind)
        qDot(ind)=-qDotMax;
    end
    q = q + qDot' * 0.1;
    q(4) = pi - q(3) - q(2);
    q(5) = 0;

    dobot.model.animate(q);
    hold on
    % delete('arrow');
    arrow = quiver3(t(1), t(2), t(3), v(1), v(2), v(3));

    cam.clf();
    cam.plot(patternCorners(q, dobot));
    cam.plot(cornerGoal, '*r');
    t = dobot.model.fkine(q).t;

    cam.hold(true);
    drawnow
    if i == 1
        % pause();
    end
end


% 
% % P1 = dobot.model.fkine(q).t;
% % disp(P1)
% P1 = [0.2; 0.2; 0.2];
% plot_sphere(P1, 0.05, 'r');
% cam.clf();
% cam.plot(P1); 
% cam.hold(true);