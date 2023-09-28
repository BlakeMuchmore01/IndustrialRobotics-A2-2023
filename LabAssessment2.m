%% Main Function
% CoBot Blackjack Table Simulation
function LabAssessment2()
    clear; clc; clf; close all; % Clearing workspace, command window, and figures
    % profile on; % Profiling the code

    % Creating log file and setting command window level
    L = log4matlab('logFile.log');
    L.SetCommandWindowLevel(L.DEBUG);

    figure(1); % Creating figure to simulate robots
    hold on; axis([-1 1 -1 1 -0.01 1]);

    %% Spawning the robots and surrounding environment
    % Spawning the Aubo i5 and associated 2F-85 gripper
    auboI5 = AuboI5(eye(4),L);
    auboI5.UpdateToolTr; % Updating end-effector transform property
    twoFingeredGripper = TwoFingeredGripper(auboI5.toolTr,L);

    twoFingeredGripper.fingerModels{1}.model.teach();

    % Spawning the Dobot Magician and associated suction gripper
    dobotMagician = DobotMagician(eye(4)*transl(0,0.3,0),L);

    %% Creating the GUI Figure
    % figure(2); % Creating figure for GUI
    % Creating the app GUI object

    pause;




end
