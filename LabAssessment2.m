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
        handOffTransform = [eul2rotm([0 0 -pi/2])*eul2rotm([90 0 0]*pi/180) [0.2222, 0.32, 0.25]'; zeros(1,3) 1];
    end
    
    %% Methods of the Class
    methods (Static)
        %% Hit Functionality
        function HitSelected(app, toDealer)
            % Getting the index for the last card in the cardModels array
            endCardIndex = app.GetEndCardIndex();

            % Getting the pose transform of the card related to the index
            % Translation matrix is applied so dobot moves just above the card
            cardTransform = app.playingCards.cardModels{endCardIndex}.base.T * transl(0,0,0.045);

            % Getting the qMatrix to move the dobot to pick up the first card
            qMatrixDobot = app.dobotMagician.GetCartesianMovement(cardTransform);

            % Looping through the qMatrix and animating the dobot
            counter = 1; % Creating counter to loop the while loop
            while counter <= size(qMatrixDobot,1)
                % Checking if the arduino estop has been hit
                if app.arduino ~= 0
                    app.RealEstopReading(app.arduino.CheckButtonPressed());
                end
                
                % Animating the robot to qMatrix pose
                app.dobotMagician.model.animate(qMatrixDobot(counter,:)); % Animating the dobot movement
                app.dobotMagician.UpdateToolTr(); % Updating the end-effector property of the dobot

                % Ensuring that the environment is safe before increasing counter variable (Continue button is
                % on, light curtain safe, no collisions)
                if LabAssessment2.LightCurtainCheck(app.hand.handModels{1}) && ...
                        app.continuePressed % && ~app.auboI5.isCollision() && ~app.dobotMagician.isCollision() 

                    drawnow; % Updating the plot
                    counter = counter + 1;
                end
            end
            app.logFile.mlog = {app.logFile.DEBUG, 'HitSelected','Dobot has picked up a card'};

            % Getting the qMatrix to move the dobot back to its original position with the card
            initialPose = app.dobotMagician.model.fkine(app.dobotMagician.defaultRealQ).T;
            qMatrixDobot = app.dobotMagician.GetCartesianMovement(initialPose);

            counter = 1; % Resetting the counter for the next movement
            % Looping through the qMatrix and animating the dobot
            while counter <= size(qMatrixDobot,1)
                % Checking if the arduino estop has been hit
                if app.arduino ~= 0
                    app.RealEstopReading(app.arduino.CheckButtonPressed());
                end
                
                % Moving the dobot magician
                app.dobotMagician.model.animate(qMatrixDobot(counter,:)); % Animating the dobot movement
                app.dobotMagician.UpdateToolTr(); % Updating the end-effector property of the dobot
                
                % Moving the card with the dobot
                app.playingCards.cardModels{endCardIndex}.base = app.dobotMagician.toolTr * transl(0,0,-0.045);
                app.playingCards.cardModels{endCardIndex}.animate(0); % Animating change in position

                % Ensuring that the environment is safe before increasing counter variable (Continue button is
                % on, light curtain safe, no collisions)
                if LabAssessment2.LightCurtainCheck(app.hand.handModels{1}) && ...
                        app.continuePressed % && ~app.auboI5.isCollision() && ~app.dobotMagician.isCollision() 

                    drawnow; % Updating the plot
                    counter = counter + 1; % Increasing 
                end
            end
            app.logFile.mlog = {app.logFile.DEBUG, 'HitSelected','Card is ready to be transitioned to Aubo i5'};

            % Getting the qMatrix to move the aubo i5 to take the card from the dobot
            qMatrixAubo = app.auboI5.GetCartesianMovement(LabAssessment2.handOffTransform);

            counter = 1; % Resetting the counter for the next movement
            % Looping through the qMatrix and animating the aubo
            while counter <= size(qMatrixAubo,1)
                % Checking if the arduino estop has been hit
                if app.arduino ~= 0
                    app.RealEstopReading(app.arduino.CheckButtonPressed());
                end

                % Moving the aubo i5
                app.auboI5.model.animate(qMatrixAubo(counter,:)); % Animating the aubo movement
                app.auboI5.UpdateToolTr(); % Updating the end-effector property of the aubo

                % Moving the gripper alongside the aubo
                for gripperNum = 1:2
                    % Updating the position of the grippers
                    app.auboI5.tool{gripperNum}.UpdateGripperPosition(app.auboI5.toolTr,gripperNum);
                end

                % Ensuring that the environment is safe before increasing counter variable (Continue button is
                % on, light curtain safe, no collisions)
                if LabAssessment2.LightCurtainCheck(app.hand.handModels{1}) && ...
                        app.continuePressed % && ~app.auboI5.isCollision() && ~app.dobotMagician.isCollision() 

                    drawnow; % Updating the plot
                    counter = counter + 1;
                end
            end

            % Closing the gripper on the aubo
            qMatrixGripper = app.auboI5.tool{1}.GetOpenCloseQMatrix();

            counter = 1; % Resetting the counter for the next movement
            % Looping through the qMatrix and animating the aubo
            while counter <= size(qMatrixGripper,1)
                % Checking if the arduino estop has been hit
                if app.arduino ~= 0
                    app.RealEstopReading(app.arduino.CheckButtonPressed());
                end
                
                % Animating the gripper's closing
                for gripperNum = 1:2
                    app.auboI5.tool{gripperNum}.model.animate(qMatrixGripper(counter,:));
                end

                % Ensuring that the environment is safe before increasing counter variable (Continue button is
                % on, light curtain safe, no collisions)
                if LabAssessment2.LightCurtainCheck(app.hand.handModels{1}) && ...
                        app.continuePressed % && ~app.auboI5.isCollision() && ~app.dobotMagician.isCollision() 

                    drawnow; % Updating the plot
                    counter = counter + 1;
                end
            end
            app.logFile.mlog = {app.logFile.DEBUG,'HitSelected','Aubo i5 collected the playing card'};

            % Determining if the card should be delt to the player or the dealer
            if toDealer
                finalCardTransforms = app.playingCards.GetFinalCardTransformsDealer(); % Getting final positions of cards
                finalTransform = finalCardTransforms(:,:,app.cardNum);
                finalTransform(3,4) = finalTransform(3,4) + 0.1;
                
                qMatrixAubo = app.auboI5.GetCartesianMovement(finalTransform);
            else 
                finalCardTransforms = app.playingCards.GetFinalCardTransforms(); % Getting final positions of cards

                % Getting the final transform for the current card and increasing its z component by 0.1 (To avoid collision)
                finalTransform = finalCardTransforms(:,:,app.cardNum,app.player);
                finalTransform(3,4) = finalTransform(3,4) + 0.1;

                qMatrixAubo = app.auboI5.GetCartesianMovement(finalTransform);
            end

            counter = 1; % Resetting the counter for the next movement
            % Looping through the qMatrix and animating the aubo
            while counter <= size(qMatrixGripper,1)
                % Checking if the arduino estop has been hit
                if app.arduino ~= 0
                    app.RealEstopReading(app.arduino.CheckButtonPressed());
                end
                
                % Animating the aubo movement
                app.auboI5.model.animate(qMatrixAubo(counter,:)); % animating the model to the next pose in the qMatrix
                app.auboI5.UpdateToolTr(); % Updating the end-effector transform property

                % Updating the positions fo the gripper fingers
                for gripperNum = 1:2
                    app.auboI5.tool{gripperNum}.UpdateGripperPosition(app.auboI5.toolTr, gripperNum);
                end

                % Updating the position of the card alongside the gripper
                app.playingCards.cardModels{endCardIndex}.base = app.auboI5.toolTr * trotz(pi/2) * trotx(pi/2) * transl(0,0.2,-0.01);
                app.playingCards.cardModels{endCardIndex}.animate(0);
                
                % Ensuring that the environment is safe before increasing counter variable (Continue button is
                % on, light curtain safe, no collisions)
                if LabAssessment2.LightCurtainCheck(app.hand.handModels{1}) && ...
                        app.continuePressed % && ~app.auboI5.isCollision() && ~app.dobotMagician.isCollision() 

                    drawnow; % Updating the plot
                    counter = counter + 1;
                end
            end

             % Determining if the card should be delt to the player or the dealer
            if toDealer
                finalCardTransforms = app.playingCards.GetFinalCardTransformsDealer(); % Getting final positions of cards
                finalTransform = finalCardTransforms(:,:,app.cardNum);
                qMatrixAubo = app.auboI5.GetCartesianMovement(finalTransform);
            else 
                finalCardTransforms = app.playingCards.GetFinalCardTransforms(); % Getting final positions of cards

                % Getting the final transform for the current card and increasing its z component by 0.1 (To avoid collision)
                finalTransform = finalCardTransforms(:,:,app.cardNum,app.player);
                qMatrixAubo = app.auboI5.GetCartesianMovement(finalTransform);
            end

            counter = 1; % Resetting the counter for the next movement
            % Looping through the qMatrix and animating the aubo
            while counter <= size(qMatrixGripper,1)
                % Checking if the arduino estop has been hit
                if app.arduino ~= 0
                    app.RealEstopReading(app.arduino.CheckButtonPressed());
                end

                app.auboI5.model.animate(qMatrixAubo(counter,:)); % animating the model to the next pose in the qMatrix
                app.auboI5.UpdateToolTr(); % Updating the end-effector transform property

                % Updating the positions fo the gripper fingers
                for gripperNum = 1:2
                    app.auboI5.tool{gripperNum}.UpdateGripperPosition(app.auboI5.toolTr, gripperNum);
                end

                % Updating the position of the card alongside the gripper
                app.playingCards.cardModels{endCardIndex}.base = app.auboI5.toolTr * trotz(pi/2) * trotx(pi/2) * transl(0,0.2,-0.01);
                app.playingCards.cardModels{endCardIndex}.animate(0);
                
                % Ensuring that the environment is safe before increasing counter variable (Continue button is
                % on, light curtain safe, no collisions)
                if LabAssessment2.LightCurtainCheck(app.hand.handModels{1}) && ...
                        app.continuePressed % && ~app.auboI5.isCollision() && ~app.dobotMagician.isCollision() 

                    drawnow; % Updating the plot
                    counter = counter + 1;
                end
            end
            app.logFile.mlog = {app.logFile.DEBUG,'HitSelected','Aubo i5 has distributed the card'};

            % Getting the qMatrix to move the aubo back to its initial pose
            qMatrixAubo = app.auboI5.ReturnAuboToInitialPose();
            qMatrixGripper = app.auboI5.tool{1}.GetOpenCloseQMatrix();

            counter = 1; % Resetting the counter for the next movement
            % Looping through the qMatrix and animating the aubo
            while counter <= size(qMatrixGripper,1)
                % Checking if the arduino estop has been hit
                try app.RealEstopReading(app.arduino.CheckButtonPressed()); end

                % Moving the aubo i5
                app.auboI5.model.animate(qMatrixAubo(counter,:)); % Animating the aubo movement
                app.auboI5.UpdateToolTr(); % Updating the end-effector property of the aubo

                % Animating the gripper's openning
                for gripperNum = 1:2
                    app.auboI5.tool{gripperNum}.UpdateGripperPosition(app.auboI5.toolTr,gripperNum);
                    app.auboI5.tool{gripperNum}.model.animate(qMatrixGripper(counter,:));
                end

                % Ensuring that the environment is safe before increasing counter variable (Continue button is
                % on, light curtain safe, no collisions)
                if LabAssessment2.LightCurtainCheck(app.hand.handModels{1}) && ...
                        app.continuePressed % && ~app.auboI5.isCollision() && ~app.dobotMagician.isCollision() 

                    drawnow; % Updating the plot
                    counter = counter + 1;
                end
            end
            app.logFile.mlog = {app.logFile.DEBUG,'HitSelected','Next card ready to be delt'};

            % Increasing the cardNum to deal next card to next position
            app.cardNum = app.cardNum + 1;
            app.NotifyCardDelt(); % Removing the delt card from the cardModels array
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
            cardstand = PlaceObject('CardStand.ply', [0.5 0.53 0.02]); %#ok<NASGU>
            cardstand = PlaceObject('CardStand.ply', [0.5 -0.2 0.02]); %#ok<NASGU>
            cardstand = PlaceObject('CardStand.ply', [0.4 0.16 0.02]); %#ok<NASGU>
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
        function isClear = LightCurtainCheck(model)
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
