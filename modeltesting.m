clf; clear; clc;
model = 2;

figure(1);

if model == 1
    % Creating log file and setting command window level
    L = log4matlab('logFile.log');
    L.SetCommandWindowLevel(L.DEBUG);
    
    % Creating 2F-85 gripper and attaching it to the Aubo i5 end-effector
    twoFingeredGripper = []; % Creating cell structure to store gripper fingers
    for i = 1:1
        twoFingeredGripper{i} = TwoFingeredGripper(eye(4),i,L);
    end
    
    twoFingeredGripper{1}.model.teach(zeros(1,5));
end

if model == 2
    % Creating log file and setting command window level
    L = log4matlab('logFile.log');
    L.SetCommandWindowLevel(L.DEBUG);

    % Creating aubo i5
    r = AuboI5(eye(4),L);
end

pause;