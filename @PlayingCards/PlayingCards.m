%% Playing Cards
classdef PlayingCards < RobotBaseClass
    %% Robot Class Properties
    % Code has been integrated from UTS' robotCow code functionality
    % https://canvas.uts.edu.au/courses/27375/pages/subject-resources?module_item_id=1290469

    % Constant Properties
    properties (Access = public, Constant)
        cardCount = 52; % Default number of playing cards to create/plot
        WORKSPACE_DIMENSIONS = [-3 3 -3 3 -0.01 -0.01]; % Dimension of workspace
        defaultName = 'PlayingCard'; % Default name for playing cards
    end

    % Non-constant properties
    properties (Access = public)
        cardModels; % Cell structure to store the playing cards created
    end

    %% ...structors
    methods
        %% Constructor for playing card objects
        function self = PlayingCards(baseTr,L)
            % Setting the default base transform if not set within
            % constructor inputs
            if nargin < 2
                % Creating log file if not supplied (indicates it hasn't
                % been created yet)
                L = log4matlab('logFile.log');
                L.SetCommandWindowLevel(L.DEBUG);

                if nargin < 1
                    % Logging that the default base transform has been used
                    L.mlog = {L.DEBUG,'PlayingCards','Base transform not set. Default base transform used'};
                    baseTr = eye(4); % Setting base transform as default
                end
            end

            % Plotting the playing cards within the workspace
            for i = 1:self.CreateModel
                % Creating the playing card D&H link model
                self.cardModels{i} = self.GetCardModel(self,[self.defaultName,num2str(i)]);
                self.cardModels{i}.base = self.cardModels{i}.base.T * baseTr; % Updating base pose of playing card

                % Plotting the playing card
                plot3d(self.cardModels{i},0,'workspace',self.WORKSPACE_DIMENSIONS,'view', ...
                    [-30,30],'delay',0,'noarrow','nowrist', 'notiles');

                % Logging creation of brick
                L.mlog = {L.DEBUG,'HalfSizedRedGreenBrick',['Brick ',num2str(i),' created within the workspace']};
            end
        end
    end

    methods (Static)
        %% Creating the playing card link models
        function model = GetCardModel(self, name)
            % Setting default name scheme for brick if no argin
            if nargin < 1
                name = self.DEFAULT_NAME;
            end

            % Reading the playing card model ply file and creating links
            [faceData,vertexData] = plyread('PlayingCard.ply','tri');
            link(1) = Link([0   0.01   0   pi   0   0]);
            model = SerialLink(link,'name',name); % Creating link model

            % Changing order of cell array from {faceData, []} to 
            % {[], faceData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.faces = {[], faceData};

            % Changing order of cell array from {vertexData, []} to 
            % {[], vertexData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.points = {[], vertexData};
        end

    end
end
