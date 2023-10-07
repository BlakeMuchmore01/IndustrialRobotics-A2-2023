%% Main Function
% CoBot Blackjack Table Simulation
function LabAssessment2()
    clear; clc; clf; close all; % Clearing workspace, command window, and figures
    % profile on; % Profiling the code

    % Creating log file and setting command window level
    L = log4matlab('logFile.log');
    L.SetCommandWindowLevel(L.DEBUG);

    % Creating a struct of constants used within the demo
    constants = struct('numFingers', 2, ...
        'axis', [-1.25 1.25 -1.25 1.25 -0.76 1.5], ...
        'auboOrigin', eye(4));

    % Creating the GUI object
    guiWindow = GUI;
    guiWindow.LoadLogFile(L);
    pause;
    L.mlog = {L.DEBUG,'LabAssessment2','GUI page generated'};

    %% Creating the robot's surrounding environment
    figure(1); % Creating figure to simulate robots
    hold on; axis(constants.axis);
    
    % Calculate the rotation matrix from auboOrigin
    rotationMatrix = constants.auboOrigin(1:3, 1:3);
    
    % Define the original concrete coordinates
    concreteX = [-1.25,-1.25;1.25,1.25];
    concreteY = [-1.25,1.25;-1.25,1.25];
    concreteZ = [-0.76,-0.76;-0.76,-0.76];
    
    % Apply the rotation to the concrete coordinates
    rotatedConcreteX = rotationMatrix(1, 1) * concreteX + rotationMatrix(1, 2) * concreteY;
    rotatedConcreteY = rotationMatrix(2, 1) * concreteX + rotationMatrix(2, 2) * concreteY;
    rotatedConcreteZ = concreteZ;
    
    % Translate the rotated coordinates based on auboOrigin
    concreteXTranslated = rotatedConcreteX + constants.auboOrigin(1, 4);
    concreteYTranslated = rotatedConcreteY + constants.auboOrigin(2, 4);
    concreteZTranslated = rotatedConcreteZ + constants.auboOrigin(3, 4);

    % Spawning the environmental objects
    table = PlaceObject('BlackjackTable.ply', [0 0 0]); %#ok<NASGU>
    eStop = PlaceObject('EStop.ply', [-0.17 -0.3 -0.2]); %#ok<NASGU>
    fireExtinguisher = PlaceObject('FireExtinguisher.ply', [-1.75 1.65 -1.3]); %#ok<NASGU>
    
    % Plot the concrete floor
    surf(concreteXTranslated, concreteYTranslated, concreteZTranslated, ...
        'CData', imread('concrete.jpg'), 'FaceColor', 'texturemap');
    
    %% Spawning the Robots and Grippers
    % Spawning the Aubo i5 and associated 2F-85 gripper
    auboI5 = AuboI5(constants.auboOrigin,L);
    auboI5.UpdateToolTr; % Updating end-effector transform property
    
    % Creating 2F-85 gripper and attaching it to the Aubo i5 end-effector
    twoFingeredGripper = []; % Creating cell structure to store gripper fingers
    for i = 1:constants.numFingers
        twoFingeredGripper{i} = TwoFingeredGripper(auboI5.toolTr,i,L);
    end

    % Spawning the Dobot Magician and associated suction gripper
    dobotMagician = DobotMagician(constants.auboOrigin*transl(0,0.3,0));

    pause;
end
