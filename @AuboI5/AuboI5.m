%% Aubo i5 Robot
classdef AuboI5 < RobotBaseClass
    %% Robot Class Properties
    % Constant Properties
    properties (Access = public, Constant)
        plyFileNameStem = 'AuboI5'; % Name stem used to find associated ply files
        initialJointAngles = [0 0 0 0 0 0]; % Default starting pose for Aubo i5
    end

    %% ...structors
    methods
        %% Constructor for Aubo i5 Robot
        function self = AuboI5(baseTr,L)
            % Setting the default base transform if not set within
            % constructor inputs
            if nargin < 1
                % Logging that the default base transform has been used
                L.mlog = {L.DEBUG,'AuboI5','Base transform not set. Default base transform used'};
                baseTr = eye(4); % Setting base transform as default
            end

            self.CreateModel(); % Creating the Aubo i5 D&H parameter model

            % Orientating the Aubo i5 within the workspace
            self.model.base = self.model.base.T * baseTr;
            self.homeQ = self.initialJointAngles; % Setting initial pose of Aubo i5

            % Logging the creation of the Aubo i5
            L.mlog = {L.DEBUG,'AuboI5','Aubo i5 object created within the workspace'};
        end

        %% D&H Parameter Serial Link Creation
        function CreateModel(self)
            % D&H parameters for the Aubo i5 model
            % DH = [THETA D A ALPHA SIGMA OFFSET]
            % https://www.aubo-cobot.com/public/i5product3?CPID=i5
            link(1) = Link([0    0.1215   0        pi/2   0   0]);
            link(2) = Link([0    0.1215   0        0      0   0]);
            link(3) = Link([0    0        0.4080  -pi/2   0   0]);
            link(4) = Link([0    0        0.1215   0      0   0]);
            link(5) = Link([0    0.3760   0        pi/2   0   0]);
            link(6) = Link([0    0.1025   0        0      0   0]);
            link(7) = Link([0    0        0.1025   0      0   0]);
            link(8) = Link([0    0.0940   0        0      0   0]);

            % Creating the serial link object
            self.model = SerialLink(link,'name',self.name);
        end
    end
end
