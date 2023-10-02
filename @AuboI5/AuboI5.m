%% Aubo i5 Robot
% https://www.aubo-cobot.com/public/i5product3?CPID=i5

classdef AuboI5 < RobotBaseClass
    %% Robot Class Properties
    % Constant Properties
    properties (Access = public, Constant)
        initialJointAngles = [0 pi/2 0 pi/2 pi/2 0]; % Default starting pose for Aubo i5
    end

    % Non-constant Properties
    properties (Access = public)
        currentJointAngles; % Current joint angles of the Aubo i5
        plyFileNameStem = 'AuboI5'; % Name stem used to find associated ply files
    end

    %% ...structors
    methods
        %% Constructor for Aubo i5 Robot
        function self = AuboI5(baseTr,L)
            % Setting the default base transform if not set within
            % constructor inputs
            if nargin < 2
                % Creating log file if not supplied (indicates it hasn't
                % been created yet)
                L = log4matlab('logFile.log');
                L.SetCommandWindowLevel(L.DEBUG);

                if nargin < 1
                    % Logging that the default base transform has been used
                    L.mlog = {L.DEBUG,'AuboI5','Base transform not set. Default base transform used'};
                    baseTr = eye(4); % Setting base transform as default
                end
            end

            self.CreateModel(); % Creating the Aubo i5 D&H parameter model

            % Orientating the Aubo i5 within the workspace
            self.model.base = self.model.base.T * baseTr;
            self.homeQ = self.initialJointAngles; % Setting initial pose of Aubo i5

            % Plotting the Aubo i5 and associated ply models
            self.model.plot(self.initialJointAngles,'noname','noshadow','nowrist','nobase','notiles');

            % Logging the creation of the Aubo i5
            L.mlog = {L.DEBUG,'AuboI5','Aubo i5 object created within the workspace'};
        end

        %% D&H Parameter Serial Link Creation
        function CreateModel(self)
            % D&H parameters for the Aubo i5 model
            % DH = [THETA D A ALPHA SIGMA OFFSET]
            % https://www.aubo-cobot.com/public/i5product3?CPID=i5
            link(1) = Link([0    0.1215   0        pi/2   0   0]);
            link(2) = Link([0    0        0.1405   0      0   0]);
            link(3) = Link([0    0        0.3760   0      0   0]);
            link(4) = Link([0   -0.1025   0        pi/2   0   0]);
            link(5) = Link([0    0.1025   0       -pi/2   0   0]);
            link(6) = Link([0    0.0940   0        0      0   0]);

            % Qlims for each joint
            % https://www.aubo-cobot.com/public/i5product3?CPID=i5
            link(1).qlim = [-175  175]*pi/180;
            link(2).qlim = [-175  175]*pi/180;
            link(3).qlim = [-175  175]*pi/180;
            link(4).qlim = [-175  175]*pi/180;
            link(5).qlim = [-175  175]*pi/180;
            link(6).qlim = [-360  360]*pi/180;

            % Creating the serial link object
            self.model = SerialLink(link,'name',self.name);
        end

        %% Updator for End Effector Transform (Tool Transform)
        function UpdateToolTr(self)
            % Updating the toolTr property of the robot
            % Used to update the pose of the gripper to the end-effector
            self.currentJointAngles = self.model.getpos(); % Getting the current joint angles of Aubo i5
            self.toolTr = self.model.fkine(self.currentJointAngles).T; % Updating toolTr property
        end

    end
end
