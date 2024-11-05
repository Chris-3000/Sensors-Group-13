function Tb2c = findTb2c(dobotSim, cam, cameraParams)
    qSim = [0,0,pi/2,pi/2, 0];
    Tb2e = dobotSim.model.fkine(qSim).T;
    Te2p = transl(0.013,0.03,0.044+0.015) * troty(pi/2) * trotz(-pi/2);

    Tc2p = zeros(4,4);
    worldPoints = generateCheckerboardPoints([5,8], 10);
    while true
        frame = snapshot(cam);
        [points, ~] = detectCheckerboardPoints(frame);
        if size(points, 1) == 28
            [rotationMatrix, translationVector] = extrinsics(points, worldPoints, cameraParams);
            Tc2p = [rotationMatrix, 0.001*translationVector'; zeros(1,3), 1];
            disp("Image taken");
            break;
        else
            disp("Not enough points detected");
        end
    end

    Tb2c = Tb2e * Te2p * inv(Tc2p);
end