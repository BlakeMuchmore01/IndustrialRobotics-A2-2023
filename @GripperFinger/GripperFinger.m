% Gripper Finger Class for 2F-85 Robotic Gripper
classdef GripperFinger < RobotBaseClass
    %% Gripper Finger Class Properties
    % Constant Properties
    properties (Access = public, Constant)
        initialJointAngles = zeros(1,5); % Default starting pose for Gripper Finger
    end

    % Non-constant Properties
    properties (Access = public)
        plyFileNameStem = 'GripperFinger'; % Name stem used to find associated ply files when plotting
    end

    %% ...structors
    methods
        %% Constructor for Gripper Finger
        function self = GripperFinger(baseTr)
            self.CreateModel(); % Creating the Gripper Finger D&H parameter model

            % Orientating the Aubo i5 within the workspace
            self.model.base = self.model.base.T * baseTr;
            self.homeQ = self.initialJointAngles; % Setting initial pose of Gripper Finger
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

            % Creating the serial link object
            self.model = SerialLink(link,'name',self.name);
        end

    end
end