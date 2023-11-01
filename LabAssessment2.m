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
        function HitSelected(guiWindow)

        end

        %% Function to deal cards to the dealer
        function DealToDealer(guiWindow)
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
