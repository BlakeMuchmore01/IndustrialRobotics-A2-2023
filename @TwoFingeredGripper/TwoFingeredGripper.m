% 2F-85 Robotiq Gripper for Aubo i5
classdef TwoFingeredGripper
    %% Gripper Class Properties
    % Non-constant Properties
    properties (Access = public)
        isClosed = false; % Variable determining if gripper is currently in open/closed state
        plyFileNameStem = 'TwoFingeredGripper'; % Name stem used to find associated ply files when plotting
    end

    % Constant Properties
    properties (Access = public, Constant)
        numFingers = 2; % Number of Fingers to attach to the gripper
        initialJointAngles = zeros(1,5); % Default starting pose for Gripper Finger
    end

    %% ...structors
    methods
        %% Constructor for 2F-85
        function self = TwoFingeredGripper(baseTr,L)
            % Setting the default base transform if not set within
            % constructor inputs
            if nargin < 2
                % Creating log file if not supplied (indicates it hasn't
                % been created yet)
                L = log4matlab('logFile.log');
                L.SetCommandWindowLevel(L.DEBUG);

                if nargin < 1
                    % Logging that the default base transform has been used
                    L.mlog = {L.DEBUG,'TwoFingeredGripper','Base transform not set. Default base transform used'};
                    baseTr = eye(4); % Setting base transform as default
                end
            end

            % Logging the creation of the Aubo i5
            L.mlog = {L.DEBUG,'TwoFingeredGripper','TwoFingeredGripper object created within the workspace'};
        end

        %% D&H Parameter Serial Link Creation
        function CreateModel(self)
            % D&H parameters for the 2F-85 finger model
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
            link(3).qlim = [10 25]*pi/180;
            link(4).qlim = [14 15]*pi/180;
            link(5).qlim = [-27 35]*pi/180;
        end

    end
end
