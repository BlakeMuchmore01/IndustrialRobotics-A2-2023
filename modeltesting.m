clf; clear; clc;
model = 4;

figure(1);

if model == 1
    % Creating log file and setting command window level
    L = log4matlab('logFile.log');
    L.SetCommandWindowLevel(L.DEBUG);
    
    % Creating 2F-85 gripper and attaching it to the Aubo i5 end-effector
    twoFingeredGripper = []; % Creating cell structure to store gripper fingers
    for i = 1:2
        twoFingeredGripper{i} = TwoFingeredGripper(eye(4),i,L);
    end
    
    twoFingeredGripper{1}.model.teach([0 45 45]*pi/180);
    twoFingeredGripper{2}.model.teach([0 45 45]*pi/180);
end

if model == 2
    % Creating log file and setting command window level
    L = log4matlab('logFile.log');
    L.SetCommandWindowLevel(L.DEBUG);

    figure(1);
    % Creating aubo i5
    r = AuboI5(eye(4),L);
    axis([-1 1 -1 1 -1 1]);

    r.model.teach([0 pi/2 0 pi/2 0 0]);
end

if model == 3
    % Creating log file and setting command window level
    L = log4matlab('logFile.log');
    L.SetCommandWindowLevel(L.DEBUG);
    


    % Creating the GUI object
    guiWindow = GUI;
    guiWindow.LoadLogFile(L); % Loading the logfile into the gui class

end

if model == 4
    
    L = log4matlab('logFile.log');
    L.SetCommandWindowLevel(L.DEBUG);

    figure(1); % Creating figure to simulate robots
    hold on; axis(LabAssessment2.axisLimits); camlight;

    LabAssessment2.CreateEnvironment(L)

    auboI5 = AuboI5(LabAssessment2.auboOrigin,L); % Spawning the Aubo i5 and associated 2F-85 gripper
    % dobotMagician = DMagician(LabAssessment2.auboOrigin*transl(0,0.5,0)); % Spawning the Dobot Magician and associated suction gripper
    
    transform = eye(4);
    transform(1:3, 4) = [0.6, 0, 0.1];
    transform(1:3,1:3) = eul2rotm([-90 0 220]*pi/180,"XYZ");

    qmat = auboI5.GetCartesianMovement(transform);
    

    for i = 1:1:size(qmat,1)
        currentTr = auboI5.model.fkine(auboI5.model.getpos()).T;

        auboI5.model.animate(qmat(i,:));
        auboI5.UpdateToolTr;
        disp(auboI5.toolTr);

        for j = 1:2
             auboI5.tool{1,j}.UpdateGripperPosition(auboI5.toolTr,j);
        end

        pause(0.01);
        drawnow;
    end



end

pause;