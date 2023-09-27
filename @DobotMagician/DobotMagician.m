%% Dobot Magician
classdef DobotMagician < RobotBaseClass
    %% Robot Class Properties
    % Constant Properties
    properties (Access = public, Constant)
        initialJointAngles = [0 pi/4 -pi/4 0]; % Default starting pose for Aubo i5
    end

    % Non-constant Properties
    properties (Access = public)
        currentJointAngles; % Current joint angles of the Aubo i5
        plyFileNameStem = 'DobotMagician'; % Name stem used to find associated ply files
    end

    %% ...structors
    methods
        %% Constructor for Dobot Magician
        function self = DobotMagician(baseTr,L)
            % Setting the default base transform if not set within
            % constructor inputs
            if nargin < 2
                % Creating log file if not supplied (indicates it hasn't
                % been created yet)
                L = log4matlab('logFile.log');
                L.SetCommandWindowLevel(L.DEBUG);

                if nargin < 1
                    % Logging that the default base transform has been used
                    L.mlog = {L.DEBUG,'DobotMagician','Base transform not set. Default base transform used'};
                    baseTr = eye(4); % Setting base transform as default
                end
            end

            self.CreateModel(); % Creating the Dobot Magician D&H parameter model

            % Orientating the Dobot Magician within the workspace
            self.model.base = self.model.base.T * baseTr;
            self.homeQ = self.initialJointAngles; % Setting initial pose of Aubo i5

            % Plotting the Dobot Magician and associated ply models
            self.model.plot(self.initialJointAngles);

            % Logging the creation of the Dobot Magician
            L.mlog = {L.DEBUG,'DobotMagician','Dobot Magician object created within the workspace'};
        end

        %% D&H Parameter Serial Link Creation
        function CreateModel(self)
            % D&H parameters for the Aubo i5 model
            % DH = [THETA D A ALPHA SIGMA OFFSET]
            % https://www.dobot-robots.com/products/education/magician.html
            link(1) = Link([0   0.1380   0        pi/2   0   0]);
            link(2) = Link([0   0        0.1350   0      0   0]);
            link(3) = Link([0   0        0.1470   pi/2   0   0]);
            link(4) = Link([0   0        0.01     0      0   0]);

            % Qlims for each joint
            % https://www.dobot-robots.com/products/education/magician.html
            link(1).qlim = [-90 90]*pi/180;
            link(2).qlim = [0   85]*pi/180;
            link(3).qlim = [-90 10]*pi/180;
            link(4).qlim = [-90 90]*pi/180;

            % Creating the serial link object
            self.model = SerialLink(link,'name',self.name);
        end

        %% Updator for End Effector Transform (Tool Transform)
        function UpdateToolTr(self)
            % Updating the toolTr property of the robot
            % Used to update the pose of the gripper to the end-effector
            self.currentJointAngles = self.model.getpos(); % Getting the current joint angles of Aubo i5
            self.toolTr = self.model.fkine(self.currentJointAngles); % Updating toolTr property
        end

    end
end
