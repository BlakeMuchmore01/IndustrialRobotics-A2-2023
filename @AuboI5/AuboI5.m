%% Aubo i5 Robot
% https://www.aubo-cobot.com/public/i5product3?CPID=i5

classdef AuboI5 < RobotBaseClass
    %% Robot Class Properties
    % Constant Properties
    properties (Access = public, Constant)
        initialJointAngles = [0 pi/2 0 pi/2 0 0]; % Default starting pose for Aubo i5
        movementSteps = 1000; % Number of steps allocated for movement trajectories
        movementTime = 10; % Time for movements undergone by the Aubo i5
        epsilon = 0.05; % Maximum measure of manipulability to then require Damped Least Squares
        movementWeight = diag([1 1 1 0.1 0.1 0.1]); % Weighting matrix for movement velocity vector
        maxLambda = 0.01;
    end

    % Non-constant Properties
    properties (Access = public)
        currentJointAngles; % Current joint angles of the Aubo i5
        tool = cell(1,2); % Variable to store the tool (2F-85 Gripper) that is used by the Aubo i5
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

            self.model.base = self.model.base.T * baseTr; % Orientating the Aubo i5 within the workspace
            self.homeQ = self.initialJointAngles; % Setting initial pose of Aubo i5

            % Plotting the Aubo i5 and associated ply models
            self.PlotAndColourRobot();

            % Logging the creation of the Aubo i5
            L.mlog = {L.DEBUG,'AuboI5','Aubo i5 object created within the workspace'};

            % Creating 2F-85 gripper and attaching it to the Aubo i5 end-effector
            self.UpdateToolTr; % Updating the end-effector transform property
            for gripperFinger = 1:2
                self.tool{gripperFinger} = TwoFingeredGripper(self.toolTr, gripperFinger, L);
            end
        end

        %% D&H Parameter Serial Link Creation
        function CreateModel(self)
            % D&H parameters for the Aubo i5 model
            % DH = [THETA D A ALPHA SIGMA OFFSET]
            % https://www.aubo-cobot.com/public/i5product3?CPID=i5
            link(1) = Link([0    0.1215   0        pi/2   0   0]);
            link(2) = Link([0    0        0.4080   0      0   0]);
            link(3) = Link([0    0        0.3760   0      0   0]);
            link(4) = Link([0   -0.1025   0        pi/2   0   0]);
            link(5) = Link([0    0.1025   0        pi/2   0   0]);
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

        %% Moving the Aubo i5 to a Desired Transform
        function MoveToCartesian(self, coordinateTransform)
            deltaT = self.movementTime/self.movementSteps; % Calculating discrete time step

            % Allocating memory to data arrays
            manipulability = zeros(self.movementSteps, 1);      % Array of measure of manipulability
            qMatrix = zeros(self.movementSteps, self.model.n);  % Array of joint angle states
            qdot = zeros(self.movementSteps, self.model.n);     % Array of joint velocities
            theta = zeros(3, self.movementSteps);               % Array of roll-pitch-yaw angles
            trajectory = zeros(3, self.movementSteps);          % Array of x-y-z trajectory

            % Getting the initial and final x-y-z coordinates
            initialTr = self.model.fkine(self.model.getpos()).T; % Getting the transform for the robot's current pose
            x1 = initialTr(1:3,4); % Extracting the x-y-z coordinates from the current pose transform
            x2 = coordinateTransform(1:3,4); % Extracting  the x-y-z coordiantes from the final pose transform

            % Getting the initial and final roll-pitch-yaw angles
            rpy1 = tr2rpy(initialTr(1:3,1:3)); % Getting the inial roll-pitch-yaw angles
            rpy2 = tr2rpy(coordinateTransform(1:3,1:3)); % Getting the final roll-pitch-yaw angles

            % Creating the movement trajectory
            s = lspb(0,1,self.movementSteps); % Create interpolation scalar
            for i = 1:self.movementSteps
                trajectory(:,i) = x1*(1-s(i)) + s(i)*x2; % Creating the translation trajectory
                theta(:,i) = rpy1*(1-s(i)) + s(i)*rpy2; % Creating the rotation trajectory
            end

            % Creating the transform for the first instance of the trajectory
            firstTr = rpy2tr(theta(1,1), theta(2,1), theta(3,1));
            firstTr(1:3,4) = trajectory(:,1); % Updating the position section of the matrix
            q0 = self.model.getpos(); % Getting the initial guess for the joint angles
            qMatrix(1,:) = self.model.ikine(firstTr, 'q0', q0); % Solving the qMatrix for the first waypointx

            % Tracking the movement trajectory with RMRC
            for i = 1:self.movementSteps-1
                currentTr = self.model.fkine(self.model.getpos()).T; % Gettign the forward transform at current joint states
                deltaX = (trajectory(:,i+1) - currentTr(1:3,4)); % Getting the position error from the next waypoint
                Rd = rpy2tr(theta(1,i+1), theta(2,i+1), theta(3,i+1)); % Gettign the next RPY angles
                Ra = currentTr(1:3,1:3); % Getting the current end-effector rotation matrix
                
                Rdot = (1/deltaT) * (Rd-Ra); % Calcualting the roll-pitch-yaw angular velocity rotation matrix
                S = 0;
                linearVelocity = (1/deltaT) * deltaX; % Calculating the linear velocities in x-y-z
                angularVelocity = 0; % Calcualting roll-pitch-yaw angular velocity

                deltaR = Rd(1:3,1:3) - Ra; % Calcualting the rotation matrix error
                deltaTheta = tr2rpy(deltaR); % Converting rotation matrix error to RPY angles
                xdot = deltaX/deltaT; % Calculate end-effector matrix to reach next waypoint

                J = self.model.jacob0(qMatrix(i,:)); % Calculating the jacobian of the current joint state

                % Implementing Damped Least Squares
                manipulability(i,1) = sqrt(det(J*J')); % Calcualting the manipulabilty of the aubo i5
                if manipulability(i,1) < self.epsilon % Checking if manipulability is within threshold
                    lambda = (1 - (manipulability(i,1)/self.epsilon)^2) * self.maxLambda; % Damping coefficient
                    invJ = inv(J'*J + lambda*eye(6))*J'; %#ok<MINV> % Apply Damped Least Squares pseudoinverse

                else % If DLS isn't required
                    invJ = inv(J);
                end

                qdot(i,:) = invJ * xdot; % Solving the RMRC equation
                qMatrix(i+1,:) = qMatrix(i,:) + delatT * qdot'; % Updating next joint state based on joint velocities
                self.model.animate(qMatrix(i+1,:));
            end
        end

    end
end
