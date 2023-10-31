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
        function HitSelected(auboI5, dobotMagician, ~, cards, cardNum, player, logFile)
            % Grabbing the last card within the card model array
            for i = size(cards.cardModels,2):-1:1
                % Checking if its empty
                if isempty(cards.cardModels{i})
                    continue; % Check next index for card
                else
                    endCard = i; % Getting the index number of the card
                    break;
                end
            end
            
            % Getting the qMatrix to move the dobot magician to the top card position
            qMatrixDobot = dobotMagician.GetCartesianMovement(cards.cardInitialTransforms{1,endCard} * transl(0,0,0.045));
            
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
                cards.cardModels{endCard}.base = dobotMagician.toolTr * transl(0, 0, -0.045);
                cards.cardModels{endCard}.animate(0);
                drawnow; % Updating the plot
            end
            logFile.mlog = {logFile.DEBUG,'HitSelected','Card is ready to be collected by Aubo i5'};

            % Getting the qMatrix to move the Aubo i5 to pick up the card
            % and getting the qMatrix to close the gripper
            qMatrixAubo = auboI5.GetCartesianMovementRMRC(LabAssessment2.handOffTransform);

            % Creating a gripper opening matrix if the gripper is currently closed
            gripperRequiresOpenning = false;
            if auboI5.tool{1}.isClosed
                qMatrixGripper = auboI5.tool{1}.GetOpenCloseQMatrix();
                gripperRequiresOpenning = true;
            end

            % Looping through the qMatrix to move the aubo to pick up the card and close the gripper 
            for i = 1:size(qMatrixAubo,1)
                % Animating the Aubo i5's movement and updating the gripper position
                auboI5.model.animate(qMatrixAubo(i,:));
                auboI5.UpdateToolTr(); % Updating the end-effector transform of the 

                % Updating the positions of the gripper fingers
                for gripperNum = 1:2
                    auboI5.tool{gripperNum}.UpdateGripperPosition(auboI5.toolTr, gripperNum);
                end

                % If the robot is currently closed, open it
                if gripperRequiresOpenning == true
                    % Looping through the qMatrix for both grippers
                    for gripperNum = 1:2
                        auboI5.tool{gripperNum}.model.animate(qMatrixGripper(i,:));
                    end
                end
                drawnow; % Updating the plot
            end
            logFile.mlog = {logFile.DEBUG,'HitSelected','Aubo i5 collected the playing card'};
            
            % Closing the gripper of the aubo
            qMatrixGripper = auboI5.tool{1}.GetOpenCloseQMatrix();
            for i = 1:size(qMatrixGripper,1)
                % Looping through the qMatrix for both grippers
                for gripperNum = 1:2
                    auboI5.tool{gripperNum}.model.animate(qMatrixGripper(i,:));
                end
                drawnow; % Updating the plot
            end

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
                cards.cardModels{endCard}.base = auboI5.toolTr * trotz(pi/2) * trotx(pi/2) * transl(0,0.2,-0.01);
                cards.cardModels{endCard}.animate(0);
                drawnow; % Updating the plot
            end
            logFile.mlog = {logFile.DEBUG,'HitSelected','Aubo i5 distributed the playing card'};

            % Removing the card that was previously moved from the intial transform array
            cards.cardModels{endCard} = [];
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
