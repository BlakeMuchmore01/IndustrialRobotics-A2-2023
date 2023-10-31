%% Dobot Magician Robot
% URL: https://en.dobot.cn/products/education/magician.html

classdef DMagician < RobotBaseClass
    %% Robot Class Properties
    % Non-constant properties
    properties (Access = public)   
        plyFileNameStem = 'DMagician'; % Name stem used to find associated ply model files
        currentJointAngles; % Current joint angles of the dobot magician
    end

    % Constant properties
    properties (Access = public, Constant)
        defaultRealQ  = [0 15 45 120 0]*pi/180; % Default joint angle
        movementSteps = 100; % Number of steps allocated for movement trajectories
        movementTime = 5; % Time for movements undergone by the Aubo i5
        epsilon = 0.1; % Maximum measure of manipulability to then require Damped Least Squares
        movementWeight = diag([1 1 1 0.1 0.1 0.1]); % Weighting matrix for movement velocity vector
        maxLambda = 0.05;
    end

    %% ...structors
    methods (Access = public) 
        %% Constructor 
        function self = DMagician(baseTr, L)
			% Setting the default base transform if not set within
            % constructor inputs
            if nargin < 2
                % Creating log file if not supplied (indicates it hasn't
                % been created yet)
                L = log4matlab('logFile.log');
                L.SetCommandWindowLevel(L.DEBUG);

                if nargin < 1
                    % Logging that the default base transform has been used
                    L.mlog = {L.DEBUG,'DMagician','Base transform not set. Default base transform used'};
                    baseTr = eye(4); % Setting base transform as default
                end
            end
            
            self.CreateModel(); % Creating the dobot magician D&H parameter model

            % Orientating the dobot magician within the workspace
            self.model.base = self.model.base.T * baseTr; % Updating the base of the robot to input base tr
            self.homeQ = self.defaultRealQ; % Setting initial starting pose of robot
            self.PlotAndColourRobot(); % Plotting the dobot magician and associated ply models

            % Logging the creation of the Dobot Magician
            L.mlog = {L.DEBUG,'DMagician','Dobot Magician object created within the workspace'};
        end

        %% D&H Parameter Serial Link Creation
        function CreateModel(self)
            % D&H parameters for the Aubo i5 model
            % DH = [THETA D A ALPHA SIGMA OFFSET]
            % URL: https://en.dobot.cn/products/education/magician.html
            link(1) = Link([0    0.103+0.0362   0       -pi/2   0    0]);
            link(2) = Link([0    0              0.135    0      0   -pi/2]);
            link(3) = Link([0    0              0.147    0      0    0]);
            link(4) = Link([0    0              0.060    pi/2   0   -pi/2]);
            link(5) = Link([0   -0.05           0        0      0    pi]);
            
            % Qlims for each joint
            link(1).qlim = [-135 135]*pi/180;
            link(2).qlim = [5    80]*pi/180;
            link(3).qlim = [-5   85]*pi/180;
            link(4).qlim = [-180 180]*pi/180;
            link(5).qlim = [-85  85]*pi/180;

            % Creating the serial link object
            self.model = SerialLink(link,'name',self.name);
        end

        %% Moving the Aubo i5 Joint to Specified Angles
        function MoveJoint(self, jointIndex, JointValue)
            % Getting the current joint angles of the Aubo i5 and updating
            % the joint value of the specified index
            self.currentJointAngles = self.model.getpos();
            self.currentJointAngles(str2double(jointIndex)) = deg2rad(JointValue);

            % Animating the robot and updating its toolTr to move the grippers
            self.model.animate(self.currentJointAngles);
            self.UpdateToolTr();
            drawnow; % Updating the plot
        end

        %% Moving the Aubo i5 End-Effector to Specified Cartesian 
        function MoveToCartesian(self, coordinate)
            % Creating the transform for the dobot magician to move to
            self.UpdateToolTr(); % Getting the end-effector transform
            rotm = self.toolTr(1:3,1:3); % Getting the rotation matrix of the end-effector
            transform = [rotm coordinate'; zeros(1,3) 1];
            
            % Getting the qMatrix to move the dobot magician to the cartesian coordiante
            qMatrix = self.GetCartesianMovement(transform);

            % Looping through the qMatrix to move the dobot magician
            for i = 1:size(qMatrix, 1)
                % Animating the dobot magician's movement and updating the gripper position
                self.model.animate(qMatrix(i,:));
                self.UpdateToolTr(); % Updating the end-effector transform of the 
                drawnow; % Updating the plot
            end
        end

        %% Function to Get Dobot to Pick Up First Card
        function GetFirstCard(self, card, cardTransform, logFile)
            % Getting the qMatrix to move the dobot to pick up the first card
            qMatrix = self.GetCartesianMovement(cardTransform);

            % Looping through the qMatrix to move the dobot to pick up the first card
            for i = 1:size(qMatrix, 1)
                % Animating the dobot's movemnet to pick up the card
                self.model.animate(qMatrix(i,:));
                drawnow; % Updating the plot
            end
            logFile.mlog = {logFile.DEBUG,'HitSelected','Dobot has picked up a card'};

            % Moving the dobot back to its upright position with the card
            initialPose = self.model.fkine(self.defaultRealQ).T;
            qMatrix = self.GetCartesianMovement(initialPose);

            % Looping through the qMatrix to move the dobot to its upright
            for i = 1:size(qMatrix, 1)
                % Animating the dobot's movemnet to pick up the card
                self.model.animate(qMatrix(i,:));
                self.UpdateToolTr; % Updating the end-effector transform property

                % Moving the card alongside the dobot's end-effector
                card.base = self.toolTr * transl(0, 0, -0.045);
                card.animate(0); % Animating the card's movement
                drawnow; % Updating the plot
            end
            logFile.mlog = {logFile.DEBUG,'HitSelected','Card is ready to be collected by Aubo i5'};
        end

        %% Updater for End Effector Transform (Tool Transform)
        function UpdateToolTr(self)
            % Updating the toolTr property of the robot
            % Used to update the pose of the gripper to the end-effector
            self.currentJointAngles = self.model.getpos(); % Getting the current joint angles of Aubo i5
            self.toolTr = self.model.fkine(self.currentJointAngles).T; % Updating toolTr property
        end

        %% Moving the DMagician to a Desired Transform
        function qMatrix = GetCartesianMovement(self, coordinateTransform)
            deltaT = self.movementTime/self.movementSteps; % Calculating discrete time step

            % Allocating memory to data arrays
            manipulability = zeros(self.movementSteps, 1);      % Array of measure of manipulability
            qMatrix = zeros(self.movementSteps, self.model.n);  % Array of joint angle states
            qdot = zeros(self.movementSteps, self.model.n);     % Array of joint velocities
            theta = zeros(3, self.movementSteps);               % Array of end-effector angles
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
            firstTr = [rpy2r(theta(1,1), theta(2,1), theta(3,1)) trajectory(:,1); zeros(1,3) 1];
            q0 = self.model.getpos(); % Getting the initial guess for the joint angles
            qMatrix(1,:) = self.model.ikcon(firstTr, q0); % Solving the qMatrix for the first waypointx

            % Tracking the movement trajectory with RMRC
            for i = 1:self.movementSteps-1
                currentTr = self.model.fkine(qMatrix(i,:)).T; % Getting the forward transform at current joint states
                deltaX = trajectory(:,i+1) - currentTr(1:3,4); % Getting the position error from the next waypoint
                Rd = rpy2r(theta(1,i+1), theta(2,i+1), theta(3,i+1)); % Getting the next RPY angles (converted to rotation matrix)
                Ra = currentTr(1:3,1:3); % Getting the current end-effector rotation matrix
                
                Rdot = (1/deltaT) * (Rd-Ra); % Calcualting the roll-pitch-yaw angular velocity rotation matrix
                S = Rdot * Ra';
                linearVelocity = (1/deltaT) * deltaX; % Calculating the linear velocities in x-y-z
                angularVelocity = [S(3,2);S(1,3);S(2,1)]; % Calcualting roll-pitch-yaw angular velocity
                xdot = self.movementWeight*[linearVelocity; angularVelocity]; % Calculate end-effector matrix to reach next waypoint

                J = self.model.jacob0(qMatrix(i,:)); % Calculating the jacobian of the current joint state

                % Implementing Damped Least Squares
                manipulability(i,1) = sqrt(det(J*J')); % Calcualting the manipulabilty of the aubo i5
                if manipulability(i,1) < self.epsilon % Checking if manipulability is within threshold
                    lambda = (1 - (manipulability(i,1)/self.epsilon)^2) * self.maxLambda; % Damping Coefficient
                    
                else % If DLS isn't required
                    lambda = 0; % Damping Coefficient
                end

                invJ = inv(J'*J + lambda * eye(self.model.n))*J'; %#ok<MINV> % DLS inverse
                qdot(i,:) = (invJ * xdot)'; % Solving the RMRC equation
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT * qdot(i,:); % Updating next joint state based on joint velocities
            end

            % Removing movements for the last link in the dobot (Suction gripper cannot rotate)
            qMatrix(:,5) = 0;
        end
    end
    
    methods(Static)
        %% RealQToModelQ
        % Convert the real Q to the model Q
        function modelQ = RealQToModelQ(realQ)
            modelQ = realQ;
            modelQ(3) = DobotMagician.ComputeModelQ3GivenRealQ2and3( realQ(2), realQ(3) );
            modelQ(4) = pi - realQ(2) - modelQ(3);    
        end
        
        %% ModelQ3GivenRealQ2and3
        % Convert the real Q2 & Q3 into the model Q3
        function modelQ3 = ComputeModelQ3GivenRealQ2and3(realQ2,realQ3)
            modelQ3 = pi/2 - realQ2 + realQ3;
        end
        
        %% ModelQToRealQ
        % Convert the model Q to the real Q
        function realQ = ModelQToRealQ( modelQ )
            realQ = modelQ;
            realQ(3) = DobotMagician.ComputeRealQ3GivenModelQ2and3( modelQ(2), modelQ(3) );
        end
        
        %% RealQ3GivenModelQ2and3
        % Convert the model Q2 & Q3 into the real Q3
        function realQ3 = ComputeRealQ3GivenModelQ2and3( modelQ2, modelQ3 )
            realQ3 = modelQ3 - pi/2 + modelQ2;
        end
    end
end