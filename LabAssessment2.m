%% Base Class to Run the Assessment
% Run the 'Main Function' to undergo the demonstration of
% both the blackjack round, and the teach functionality demo

classdef LabAssessment2 < handle
    %% Properties of the Class
    % Constant properties
    properties (Access = private, Constant)
        axis = [-1.25 1.25 -1.25 1.25 -0.76 1.5]; % Default axis of the plotting figure
        numFingers = 2; % Max number of fingers to spawn on the gripper finger
        auboOrigin = eye(4); % Default spawn location of the aubo i5 - remaining environment spawns around it
        lightCurtainCenter = [0 0 0]; % Default centre for the light curtain ellipse
        lightCurtainRadii = [1.05, 1.15, 1.1]; % Default radius' for the light curtain ellipse
    end
    
    %% Methods of the Class
    methods (Static)
        function MainFunction()
            clear; clc; clf; close all; % Clearing workspace, command window, and figures
            % profile on; % Profiling the code

            % Creating log file and setting command window level
            L = log4matlab('logFile.log');
            L.SetCommandWindowLevel(L.DEBUG); % Setting the log level to debug

            % Creating the GUI object
            guiWindow = GUI;
            guiWindow.LoadLogFile(L); % Loading the logfile into the gui class
            L.mlog = {L.DEBUG,'LabAssessment2','GUI page generated'}; % Logging creation of gui

            % Creating the figure showing to show the main demo
            figure(1); % Creating figure to simulate robots
            hold on; axis(LabAssessment2.axis); camlight;

            % Creating the environment surrounding the aubo i5
            LabAssessment2.CreateEnvironment(LabAssessment2.auboOrigin, L);

            % Creating all dynamic components for the demo (robots, cards, collision objects)
            hand = Hand(LabAssessment2.auboOrigin*transl(0,0,-1.15),L); % Hand used to verify light curtain
            cards = PlayingCards(LabAssessment2.auboOrigin*transl(0.2,0.4,0.01),L); % Spawning the cards that will be moved by robots

            auboI5 = AuboI5(LabAssessment2.auboOrigin,L); % Spawning the Aubo i5 and associated 2F-85 gripper
            dobotMagician = DMagician(LabAssessment2.auboOrigin*transl(0, 0.3, 0)); % Spawning the Dobot Magician and associated suction gripper
            
            % Using a timer to create an idler to wait to recieve incoming event requests from the GUI
            timerObj = timer('TimerFcn',@(~,~) idleFunction(), 'Period', 1, 'ExecutionMode', 'fixedRate');
            start(timerObj); % Starting  the idler function

            guiWindow.delete(); % Closing the GUI wind
        end

        %% Creating the Surrounding Environment Relative to the Aubo i5
        function CreateEnvironment(auboOrigin, logFile)
            table = PlaceObject('BlackjackTable.ply', [0 0 0]); %#ok<NASGU>
            cardstand = PlaceObject('CardStand.ply', [0.5 0.16 0.02]); %#ok<NASGU>
            cardstand = PlaceObject('CardStand.ply', [0.35 0.63 0.02]); %#ok<NASGU>
            cardstand = PlaceObject('CardStand.ply', [0.35 -0.3 0.02]); %#ok<NASGU>
            cardstand = PlaceObject('CardStand.ply', [0.2 0.16 0.02]); %#ok<NASGU>
            eStop = PlaceObject('EStop.ply', [-0.17 -0.3 -0.2]); %#ok<NASGU>
            fireExtinguisher = PlaceObject('FireExtinguisher.ply', [-1.75 1.65 -1.3]); %#ok<NASGU>

            % Define the concrete coordinates
            concreteX = [-1.25,-1.25;1.25,1.25];
            concreteY = [-1.25,1.25;-1.25,1.25];
            concreteZ = [-0.76,-0.76;-0.76,-0.76];
            
            % Plotting the concrete floor
            surf(concreteX, concreteY, concreteZ, ...
                'CData', imread('concrete.jpg'), 'FaceColor', 'texturemap');

            % Logging creation of safety environment
            logFile.mlog = {L.DEBUG,'CreateEnvironment','Safety environment created'};
        end

        %% Checking if there is Something within the Light Curtain
        function isClear = LightCurtainCheck(model)
            isClear = true; % Setting default case of function as false
            
            % Looping through the models that have been supplied
            for i = 1:size(models,2)
                % Getting the points of the model that need to be checked
                points = [model{i}.points{1,2}(:,1), model{i}.points{1,2}(:,2), model{i}.points{1,2}(:,3)];
    
                % Checking the algerbraic distance of these points
                algerbraicDist = LabAssessment2.GetAlgebraicDist(points, LabAssessment2.lightCurtainCenter, LabAssessment2.lightCurtainRadii);
                    
                % Checking if the model is within the light curtain (i.e. there
                % is an algerbraic distance of < 1 with any of the above points
                if(find(algerbraicDist < 1) > 0)
                    isClear = false; % Object has been detected within the light curtain
                    return; % Returning on first detection of object within the light curtain
                end
            end
        end
        
        %% Getter for the Algerbraic Distance between Objects and Light Curtain
        function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)
            algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
                  + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
                  + ((points(:,3)-centerPoint(3))/radii(3)).^2;
        end

    end
end
