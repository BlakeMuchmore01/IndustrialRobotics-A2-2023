% 2F-85 Robotiq Gripper for Aubo i5
classdef TwoFingeredGripper
    %% Gripper Class Properties
    % Non-constant Properties
    properties (Access = public)
        isClosed = false; % Variable determining if gripper is currently in open/closed state
        fingerModels = []; % Cell structure to store array of gripper fingers
    end

    % Constant Properties
    properties (Access = public, Constant)
        numFingers = 2; % Number of Fingers to attach to the gripper
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

            % Creating the two gripper finger objects
            for i = 1:self.numFingers
                % Creating the gripper finger object
                gripperFinger = GripperFinger(baseTr);
                self.fingerModels{i} = gripperFinger; % Storing object into models array

                % Rotating the second finger by 180 deg in z-axis
                if i == 2
                    self.fingerModels{2}.model.base = self.fingerModels{2}.model.base.T * trotz(pi);
                end

                % Plotting the Gripper Finger and associated ply models
                self.fingerModels{i}.model.plot(self.fingerModels{i}.initialJointAngles,'noname','noshadow','notiles','noarrows');
            end

            % Logging the creation of the Aubo i5
            L.mlog = {L.DEBUG,'TwoFingeredGripper','TwoFingeredGripper object created within the workspace'};
        end

    end
end
