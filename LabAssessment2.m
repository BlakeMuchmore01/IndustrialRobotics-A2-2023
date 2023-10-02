%% Main Function
% CoBot Blackjack Table Simulation
function LabAssessment2()
    clear; clc; clf; close all; % Clearing workspace, command window, and figures
    % profile on; % Profiling the code

    % Creating log file and setting command window level
    L = log4matlab('logFile.log');
    L.SetCommandWindowLevel(L.DEBUG);

    % Creating the GUI object
    guiWindow = GUI;
    L.mlog = {L.DEBUG,'LabAssessment2','GUI page generated'};

    %% Spawning the robots and surrounding environment
    figure(1); % Creating figure to simulate robots
    hold on; axis([-1 1 -1 1 -0.01 1]);

    % Spawning the Aubo i5 and associated 2F-85 gripper
    auboI5 = AuboI5(eye(4),L);
    auboI5.UpdateToolTr; % Updating end-effector transform property
    
    % Creating 2F-85 gripper and attaching it to the Aubo i5 end-effector
    twoFingeredGripper = []; % Creating cell structure to store gripper fingers
    for i = 1:2
        twoFingeredGripper{i} = TwoFingeredGripper(auboI5.toolTr,i,L);
    end

    % Spawning the Dobot Magician and associated suction gripper
    dobotMagician = DobotMagician(eye(4)*transl(0,0.3,0));

    pause;
end
