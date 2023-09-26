%% Main Function
% CoBot Blackjack Table Simulation
function LabAssessment2()
    clear; clc; clf; close all; % Clearing workspace, command window, and figures
    % profile on; % Profiling the code

    % Creating log file and setting command window level
    L = log4matlab('logFile.log');
    L.SetCommandWindowLevel(L.DEBUG);

    figure(1); % Creating figure to simulate robots
    % Spawning the robots and surrounding environment
    auboI5 = AuboI5(eye(4),L);
    auboI5.model.teach();
   

    % figure(2); % Creating figure for GUI
    % Creating the app GUI object

    pause;






end
