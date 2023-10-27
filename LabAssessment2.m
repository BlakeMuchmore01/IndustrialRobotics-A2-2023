%% Base Class to Run the Assessment
% Run the 'Main Function' to undergo the demonstration of
% both the blackjack round, and the teach functionality demo

classdef LabAssessment2 < handle
    %% Properties of the Class
    % Constant properties
    properties (Access = public, Constant)
        axisLimits = [-1.25 1.25 -1.25 1.25 -0.76 1.5]; % Default axis of the plotting figure
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

            % Creating listeners to wait for notifications from GUI
            LabAssessment2.CreateListeners(guiWindow);

            % Creating the figure showing to show the main demo
            figure(1); % Creating figure to simulate robots
            hold on; axis(LabAssessment2.axisLimits); camlight;

            % Creating the environment surrounding the aubo i5
            LabAssessment2.CreateEnvironment(L);

            % Creating all dynamic components for the demo (robots, cards, collision objects)
            % hand = Hand(LabAssessment2.auboOrigin*transl(0,0,-1.15),L); % Hand used to verify light curtain
            cards = PlayingCards(LabAssessment2.auboOrigin*transl(0.25,0.5,0.01),L); % Spawning the cards that will be moved by robots

            auboI5 = AuboI5(LabAssessment2.auboOrigin,L); % Spawning the Aubo i5 and associated 2F-85 gripper
            dobotMagician = DMagician(LabAssessment2.auboOrigin*transl(0,0.5,0)); % Spawning the Dobot Magician and associated suction gripper
            guiWindow.LoadRobots(auboI5, dobotMagician);

            % Waiting until a demonstration mode is chosen to continue with demo
            while (guiWindow.simulationMode ~= "Blackjack" && guiWindow.simulationMode ~= "Teach")
                pause(0.1);
                drawnow;
            end

            %% DEMO CODE FOR PROMOTIONAL VIDEO
            model = 1;
            if model == 1
                % Creating number to specify which player to distribute the card to
                player = 1;

                LabAssessment2.HitSelected(auboI5,dobotMagician,guiWindow,cards,1,player,L);
                LabAssessment2.HitSelected(auboI5,dobotMagician,guiWindow,cards,2,player,L);
                LabAssessment2.StandSelected(player,L);
                
                LabAssessment2.HitSelected(auboI5,dobotMagician,guiWindow,cards,1,player,L);
                LabAssessment2.HitSelected(auboI5,dobotMagician,guiWindow,cards,2,player,L);
                LabAssessment2.StandSelected(player,L);
                
                LabAssessment2.HitSelected(auboI5,dobotMagician,guiWindow,cards,1,player,L);
                LabAssessment2.HitSelected(auboI5,dobotMagician,guiWindow,cards,2,player,L);

            elseif model == 2
                % Blackjack and teach functionality
                while ~guiWindow.endDemonstration
                    % Getting the robot that is being controlled by the user
                    drawnow;
                    switch guiWindow.RobotDropDown.Value
                        case "Aubo I5"
                            % Getting the updated joint angles and animating
                            auboJointAngles = guiWindow.moveAuboDeg;
                            auboI5.model.animate(deg2rad(auboJointAngles));
                            drawnow;
                        case "Dobot Magician"
                            % Getting the updated joint angles and animating
                            dobotJointAngles = guiWindow.moveDobotDeg;
                            dobotMagician.model.animate(deg2rad(dobotJointAngles));
                            drawnow;
                    end
                    drawnow;
                end
            end

            guiWindow.delete(); % Closing the GUI wind
            close(figure(1)); % Closing the demo figure
        end

        %% Hit Functionality
        function HitSelected(auboI5, dobotMagician, ~, cards, cardNum, player, logFile)
            % Getting the qMatrix to move the dobot magician to the top card position
            qMatrixDobot = dobotMagician.GetCartesianMovement(cards.cardInitialTransforms(end));
            
            % Looping through the qMatrix to move the dobot to pick up the top card in the deck
            for i = 1:size(qMatrixDobot,1)
                % Animating the dobot's movement to pick up the card
                dobotMagician.model.animate(qMatrixDobot(i,:));
                drawnow; % Updating the plot
            end
            logFile.mlog = {logFile.DEBUG,'HitSelected','Dobot has picked up a card'};

            % Getting the qMatrix to move the dobot to its upright position
            % to hand the card to the aubo i5
            initialPosition = dobotMagician.model.fkine(dobotMagician.defaultRealQ).T;
            qMatrixDobot = dobotMagician.GetCartesianMovement(initialPosition);

            % Looping through the qMatrix to move the dobot to its upright position while holding the card
            for i = 1:size(qMatrixDobot,1)
                % Animating the dobot's movement to pick up the card
                dobotMagician.model.animate(qMatrixDobot(i,:));
                dobotMagician.UpdateToolTr()

                % Moving the card alongside the end-effector of the dobot
                cards.cardModels(end).base = dobotMagician.toolTr * trotz(pi/2) * transl(0, 0, -0.045);
                drawnow; % Updating the plot
            end
            logFile.mlog = {logFile.DEBUG,'HitSelected','Card is ready to be collected by Aubo i5'};

            % Getting the qMatrix to move the Aubo i5 to pick up the card
            % and getting the qMatrix to close the gripper
            qMatrixAubo = auboI5.GetCartesianMovementRMRC(cards.cardModels(end).base);
            qMatrixGripper = auboI5.tool{1}.GetOpenCloseQMatrix();

            % Looping through the qMatrix to move the aubo to pick up the card and close the gripper 
            for i = 1:size(qMatrixAubo,1)
                % Animating the Aubo i5's movement and updating the gripper position
                auboI5.model.animate(qMatrixAubo(i,:));
                auboI5.UpdateToolTr(); % Updating the end-effector transform of the 

                % Updating the positions of the gripper fingers
                for gripperNum = 1:2
                    auboI5.tool{gripperNum}.UpdateGripperPosition(auboI5.toolTr, gripperNum);
                end

                % Closing the gripper in the last 100 steps of the Aubo's movement     
                if (size(qMatrixAubo,1) - i <= 100)
                    % Undergoing the closing gripper qMatrix
                    for gripperNum = 1:2
                        auboI5.tool{gripperNum}.model.animate(qMatrixGripper(i,:));
                    end
                end
                drawnow; % Updating the plot
            end
            logFile.mlog = {logFile.DEBUG,'HitSelected','Aubo i5 collected the playing card'};

            % Getting the qMatrix to move the Aubo i5 to distrubute the card to the player
            % Getting the final position of the card being distributed
            finalCardTransforms = cards.GetFinalCardTransforms();
            qMatrixAubo = auboI5.GetCartesianMovementRMRC(finalCardTransforms(:,:,cardNum,player));

            % Looping through the qMatrix to move the aubo to distribute the card to the player
            for i = 1:size(qMatrixAubo,1)
                % Animating the Aubo i5's movement and updating the gripper position
                auboI5.model.animate(qMatrixAubo(i,:));
                auboI5.UpdateToolTr(); % Updating the end-effector transform of the 

                % Updating the positions of the gripper fingers
                for gripperNum = 1:2
                    auboI5.tool{gripperNum}.UpdateGripperPosition(auboI5.toolTr, gripperNum);
                end

                % Updating the position of the card alongside the gripper
                cards.cardModels(end).base = auboI5.toolTr * trotz(pi/2) * trotx(pi/2) * transl(0, 0.2, -0.01);
                drawnow; % Updating the plot
            end
            logFile.mlog = {logFile.DEBUG,'HitSelected','Aubo i5 distributed the playing card'};

            % Getting the qMatrix to move the aubo i5 to it's original position
            initialPosition = auboI5.model.fkine(auboI5.initialJointAngles).T;
            qMatrixAubo = auboI5.GetCartesianMovementRMRC(initialPosition);
            qMatrixGripper = auboI5.tool{1}.GetOpenCloseQMatrix();

            % Looping through the qMatrix to move the aubo to it's original position
            % Looping through the qMatrix to move the aubo to distribute the card to the player
            for i = 1:size(qMatrixAubo,1)
                % Animating the Aubo i5's movement and updating the gripper position
                auboI5.model.animate(qMatrixAubo(i,:));
                auboI5.UpdateToolTr(); % Updating the end-effector transform of the 

                % Updating the positions of the gripper fingers
                for gripperNum = 1:2
                    auboI5.tool{gripperNum}.UpdateGripperPosition(auboI5.toolTr, gripperNum);

                    % Opening the gripper on the first 100 steps of the aubo's motion
                    if i <= 100
                        auboI5.tool{gripperNum}.model.animate(qMatrixGripper(i,:));
                    end
                end
                drawnow;
            end
            logFile.mlog = {logFile.DEBUG,'HitSelected','Aubo i5 returned to original position'};
        end

        %% Stand Functionality
        function playerNum = StandSelected(playerNumValue, logFile)
            % Increasing playerNum so next player can undergo the blackjack functionality
            playerNum = playerNumValue + 1;
            logFile.mlog = {logFile.DEBUG,'StandSelected','User has chosen to stand'};
        end
        
        %% Creating the Surrounding Environment Relative to the Aubo i5
        function CreateEnvironment(logFile)
            table = PlaceObject('BlackjackTable.ply', [0 0 0]); %#ok<NASGU>
            cardstand = PlaceObject('CardStand.ply', [0.65 0.16 0.02]); %#ok<NASGU>
            cardstand = PlaceObject('CardStand.ply', [0.5 0.63 0.02]); %#ok<NASGU>
            cardstand = PlaceObject('CardStand.ply', [0.5 -0.3 0.02]); %#ok<NASGU>
            cardstand = PlaceObject('CardStand.ply', [0.35 0.16 0.02]); %#ok<NASGU>
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
            logFile.mlog = {logFile.DEBUG,'CreateEnvironment','Safety environment created'};
        end

        %% Function to Create Listeners to Event Based GUI Code
        function CreateListeners(guiWindow)
            % Creating listeners for events that are undergone in the GUI
            addlistener(guiWindow,'StartButtonPressed', @(src,event) LabAssessment2.StartButtonPressed(src,event));
            addlistener(guiWindow,'EmergencyButtonPressed', @(src,event) LabAssessment2.EmergencyButtonPressed(src,event));
            addlistener(guiWindow,'DemonstrationModeChanged', @(src,event) LabAssessment2.DemonstrationModeChanged(src,event));
            addlistener(guiWindow,'BlackjackButtonRequest', @(src,event) LabAssessment2.BlackjackButtonRequest(src,event));
            addlistener(guiWindow,'CartesianCoordSent', @(src,event) LabAssessment2.CartesianCoordSent(src,event));
            addlistener(guiWindow,'CartesianCoordCancelled', @(src,event) LabAssessment2.CartesianCoordCancelled(src,event));
            addlistener(guiWindow,'JointMovementSent', @(src,event) LabAssessment2.JointMovementSent(src,event));
        end

        %% Checking if there is Something within the Light Curtain
        function isClear = LightCurtainCheck(model)
            isClear = false; % Setting default case of function as false
            
            % Getting the points of the model that need to be checked
            points = [model.points{1,2}(:,1), model.points{1,2}(:,2), model.points{1,2}(:,3)];

            % Checking the algerbraic distance of these points
            algerbraicDist = LabAssessment2.GetAlgebraicDist(points, LabAssessment2.lightCurtainCenter, LabAssessment2.lightCurtainRadii);
                
            % Checking if the model is within the light curtain (i.e. there
            % is an algerbraic distance of < 1 with any of the above points
            if(find(algerbraicDist < 1) > 0)
                isClear = true; % Object has been detected within the light curtain
                return; % Returning on first detection of object within the light curtain
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
