%% 2F-140 Robotiq Gripper for Aubo i5
% https://robotiq.com/products/2f85-140-adaptive-robot-gripper

classdef TwoFingeredGripper < RobotBaseClass
    %% Gripper Class Properties
    % Non-constant Properties
    properties (Access = public)
        isClosed = false; % Variable determining if gripper finger is currently in open/closed state
        plyFileNameStem = 'TwoFingeredGripper'; % Name stem used to find associated ply files when plotting
    end

    % Constant Properties
    properties (Access = public, Constant)
        initialJointAngles = zeros(1,5); % Default starting pose for Gripper Finger
    end

    %% ...structors
    methods
        %% Constructor for 2F-140
        function self = TwoFingeredGripper(baseTr,fingerNum,L)
            % Setting the default base transform if not set within
            % constructor inputs
            if nargin < 3
                % Creating log file if not supplied (indicates it hasn't
                % been created yet)
                L = log4matlab('logFile.log');
                L.SetCommandWindowLevel(L.DEBUG);

                if nargin < 2
                    % Logging that the default base transform has been used
                    L.mlog = {L.DEBUG,'TwoFingeredGripper','Base transform not set. Default base transform used'};
                    baseTr = eye(4); % Setting base transform as default
                    
                    % Logging that the default finger number has been used
                    L.mlog = {L.DEBUG,'TwoFingeredGripper','Finger number not set. Default finger number used'};
                    fingerNum = 1; % Setting default finger number
                end
            end

            self.CreateModel(); % Creating the 2F-140 D&H parameter model

            % Orientating the Aubo i5 within the workspace
            self.model.base = self.model.base.T * baseTr;
            self.homeQ = self.initialJointAngles; % Setting initial pose of Aubo i5

            % Rotating the second finger generated by 180 deg
            if fingerNum == 2
                self.model.base = self.model.base.T * trotz(pi);
            end

            % Plotting the 2F-140 and associated ply models
            self.model.plot(self.initialJointAngles,'noname','noshadow','notiles','nowrist','nobase');
            self.PlotAndColourRobot();

            % Logging the creation of the gripper finger
            L.mlog = {L.DEBUG,'TwoFingeredGripper',['Gripper Finger ',num2str(fingerNum),' created within the workspace']};
            
            % Logging creation of 2F-140 gripper
            if fingerNum == 2
                L.mlog = {L.DEBUG,'TwoFingeredGripper','2F-85 gripper object created within the workspace'};
            end
        end

        %% D&H Parameter Serial Link Creation
        function CreateModel(self)
            % D&H parameters for the 2F-140 finger model
            % DH = [THETA D A ALPHA SIGMA OFFSET]
            % https://robotiq.com/products/2f85-140-adaptive-robot-gripper
            link(1) = Link([0    0.04926     0         pi/2    0     0]);
            link(2) = Link([0    0           0.059695  0       0     0]);
            link(3) = Link([0    0           0.08174   0       0     50*pi/180]);
            link(4) = Link([0    0           0.01445   0       0     50*pi/180]);
            link(5) = Link([0    0           0.07006   0       0    -45*pi/180]);

            % Incorporating joint limits 
            % Both finger types have same limits
            link(1).qlim = [0 0];
            link(2).qlim = [0 25]*pi/180;
            link(3).qlim = [0 25]*pi/180;
            link(4).qlim = [14 15]*pi/180;
            link(5).qlim = [-27 35]*pi/180;

            % Creating the serial link object
            self.model = SerialLink(link,'name',self.name);
        end

    end
end
