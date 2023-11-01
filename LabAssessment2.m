%% Base Class to Run the Assessment
% Run the 'Main Function' to undergo the demonstration of
% both the blackjack round, and the teach functionality demo

classdef LabAssessment2 < handle
    %% Properties of the Class
    % Constant properties
    properties (Access = public, Constant)
        lightCurtainCenter = [0 0 0]; % Default centre for the light curtain ellipse
        lightCurtainRadii = [1.05, 1.15, 1.1]; % Default radius' for the light curtain ellipse
        
        % Hand off transform (tranform aubo achieves to grab card off dobot
        handOffTransform = [eul2rotm([0 1.5708 -1.5708]) [0.2222, 0.32, 0.26]'; zeros(1,3) 1];
    end
    
    %% Methods of the Class
    methods (Static)
        %% Hit Functionality
        function HitSelected(auboI5, dobotMagician, guiWindow, cards, cardNum, player, logFile)
            % Getting the end card index within the cardModels array (i.e. one
            % that hasn't been delt prior)
            endCardIndex = guiWindow.GetEndCardIndex();
            nextCardIndex = endCardIndex-1; % Getting the index number for the next card

            % Getting the qMatrix to move the Aubo i5 to pick up the card
            % and getting the qMatrix to close the gripper
            qMatrixAubo = auboI5.GetCartesianMovement(LabAssessment2.handOffTransform);

            % Looping through the qMatrix to move the aubo to pick up the card and close the gripper 
            for i = 1:size(qMatrixAubo,1)
                %%%%%%%%%%%%%%%% CHECK COLLISION, LIGHT CURTAIN, ESTOP %%%%%%%%%%%%%%%
                
                % Animating the Aubo i5's movement and updating the gripper position
                auboI5.model.animate(qMatrixAubo(i,:));
                auboI5.UpdateToolTr(); % Updating the end-effector transform of the 

                LabAssessment2.LightCurtainCheck(guiWindow.hand.handModels{1}, guiWindow);

                % Updating the positions of the gripper fingers
                for gripperNum = 1:2
                    auboI5.tool{gripperNum}.UpdateGripperPosition(auboI5.toolTr, gripperNum);
                end
                drawnow;
            end
            
            % Closing the gripper of the aubo
            qMatrixGripper = auboI5.tool{1}.GetOpenCloseQMatrix();
            for i = 1:size(qMatrixGripper,1)
                %%%%%%%%%%%%%%%% CHECK COLLISION, LIGHT CURTAIN, ESTOP %%%%%%%%%%%%%%%
                LabAssessment2.LightCurtainCheck(guiWindow.hand.handModels{1}, guiWindow);
                % Looping through the qMatrix for both grippers
                for gripperNum = 1:2
                    auboI5.tool{gripperNum}.model.animate(qMatrixGripper(i,:));
                end
                drawnow; % Updating the plot
            end
            logFile.mlog = {logFile.DEBUG,'HitSelected','Aubo i5 collected the playing card'};

            % Getting the qMatrix to animate the Aubo i5 distributing the card to the player
            % Simultaneously move the dobot to start picking the next cards
            finalCardTransforms = cards.GetFinalCardTransforms(); % Getting the final poses of the cards
            qMatrixAubo = auboI5.GetCartesianMovement(finalCardTransforms(:,:,cardNum,player));
            qMatrixDobot = dobotMagician.GetCartesianMovement(cards.cardInitialTransforms{1,nextCardIndex} * transl(0,0,0.045));

            % Looping through the qMatrix's to distrubite the current card and pick up the next card
            for i = 1:size(qMatrixAubo, 1)
                %%%%%%%%%%%%%%%% CHECK COLLISION, LIGHT CURTAIN, ESTOP %%%%%%%%%%%%%%%
                auboI5.model.animate(qMatrixAubo(i,:)); % animating the model to the next pose in the qMatrix
                auboI5.UpdateToolTr(); % Updating the end-effector transform property


                LabAssessment2.LightCurtainCheck(guiWindow.hand.handModels{1}, guiWindow);

                % Updating the positions fo the gripper fingers
                for gripperNum = 1:2
                    auboI5.tool{gripperNum}.UpdateGripperPosition(auboI5.toolTr, gripperNum);
                end

                % Updating the position of the card alongside the gripper
                cards.cardModels{endCardIndex}.base = auboI5.toolTr * trotz(pi/2) * trotx(pi/2) * transl(0,0.2,-0.01);
                cards.cardModels{endCardIndex}.animate(0);

                dobotMagician.model.animate(qMatrixDobot(i,:)) % Animating the dobot
                dobotMagician.UpdateToolTr(); % Updating the end-effector transform property
                drawnow; % Updating the plot
            end
            logFile.mlog = {logFile.DEBUG,'HitSelected','Aubo i5 distributed the playing card'};
                
            guiWindow.NotifyCardDelt(); % Notifying that a card has been delt

            % Moving the aubo i5 to its original position and raise the dobot with the next card
            qMatrixAubo = auboI5.GetCartesianMovement(auboI5.model.fkine(auboI5.initialJointAngles).T);
            qMatrixGripper = auboI5.tool{1}.GetOpenCloseQMatrix(); % Getting qMatrix to open gripper
            qMatrixDobot = dobotMagician.GetCartesianMovement(dobotMagician.model.fkine(dobotMagician.defaultRealQ).T);

            % Looping through the qMatrix's
            for i = 1:size(qMatrixAubo)
                %%%%%%%%%%%%%%%% CHECK COLLISION, LIGHT CURTAIN, ESTOP %%%%%%%%%%%%%%%
                arduinoEstop = Arduino.CheckButtonPressed(app.arduino);

                auboI5.model.animate(qMatrixAubo(i,:)); % animating the model to the next pose in the qMatrix
                auboI5.UpdateToolTr(); % Updating the end-effector transform property
                
                LabAssessment2.LightCurtainCheck(guiWindow.hand.handModels{1}, guiWindow);


                % Updating the positions fo the gripper fingers
                for gripperNum = 1:2
                    auboI5.tool{gripperNum}.UpdateGripperPosition(auboI5.toolTr, gripperNum);
                    
                    % Opening the gripper within the first 100 steps
                    if i <= 100
                        auboI5.tool{gripperNum}.model.animate(qMatrixGripper(i,:));
                    end
                end

                dobotMagician.model.animate(qMatrixDobot(i,:)); % Animating the model to the next pose in the qMatrix
                dobotMagician.UpdateToolTr; % Updating the end-effector transform property
                
                % Updating the position of the card alongside the dobot
                cards.cardModels{nextCardIndex}.base = dobotMagician.toolTr * transl(0, 0, -0.045);
                cards.cardModels{nextCardIndex}.animate(0);
                drawnow; % Updating the plot
            end
        end

        %% Stand Functionality
        function [playerNum, cardNum] = StandSelected(playerNumValue, logFile)
            % Increasing playerNum so next player can undergo the blackjack functionality
            playerNum = playerNumValue + 1;
            cardNum = 3; % Card number defaults to 3 because next player is yet to recieve 3rd card
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
            concreteX = [-1,    -1;     1.5,   1.5];
            concreteY = [-1.25,  1.25; -1.25,  1.25];
            concreteZ = [-0.76, -0.76; -0.76, -0.76];
            
            % Plotting the concrete floor
            surf(concreteX, concreteY, concreteZ, ...
                'CData', imread('concrete.jpg'), 'FaceColor', 'texturemap');

            % Logging creation of safety environment
            logFile.mlog = {logFile.DEBUG,'CreateEnvironment','Safety environment created'};
        end

        %% Checking if there is Something within the Light Curtain
        function isClear = LightCurtainCheck(model, app)
            isClear = true; % Setting default case of function as false
            
            % Getting the points of the model that need to be checked
            points = [model.points{1,2}(:,1), model.points{1,2}(:,2), model.points{1,2}(:,3)];
            points = points(1:3) + model.base.t(1:3)';

            % Checking the algerbraic distance of these points
            algerbraicDist = LabAssessment2.GetAlgebraicDist(points, LabAssessment2.lightCurtainCenter, LabAssessment2.lightCurtainRadii);
                
            % Checking if the model is within the light curtain (i.e. there
            % is an algerbraic distance of < 1 with any of the above points
            if(find(algerbraicDist < 1) > 0)
                isClear = false; % Object has been detected within the light curtain
                app.eStopPause;
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
