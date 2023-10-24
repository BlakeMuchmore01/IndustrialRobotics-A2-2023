%% Aubo i5 Robot
% https://www.aubo-cobot.com/public/i5product3?CPID=i5

classdef AuboI5 < RobotBaseClass
    %% Robot Class Properties
    % Constant Properties
    properties (Access = public, Constant)
        initialJointAngles = [0 135 -100 145 -90 0]*pi/180; % Default starting pose for Aubo i5
        movementSteps = 1000; % Number of steps allocated for movement trajectories
        movementTime = 10; % Time for movements undergone by the Aubo i5
        epsilon = 0.01; % Maximum measure of manipulability to then require Damped Least Squares
        movementWeight = diag([0.8 0.8 0.8 0.2 0.2 0.2]); % Weighting matrix for movement velocity vector
        maxLambda = 0.05; % Value used for Damped Least Squares
    end

    % Non-constant Properties
    properties (Access = public)
        currentJointAngles; % Current joint angles of the Aubo i5
        tool = cell(1,2); % Variable to store the tool (2F-85 Gripper) that is used by the Aubo i5
        plyFileNameStem = 'AuboI5'; % Name stem used to find associated ply files
        linkCentres = cell(6,3); % Structure of link centres to use for ellipsoid updating
        linkRadii = cell(6,3); % Structure of link elliposid radii
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
            self.ellipsis.base = self.ellipsis.base.T * baseTr;
            self.homeQ = self.initialJointAngles; % Setting initial pose of Aubo i5

            % Plotting the Aubo i5 and associated ply models
            self.PlotAndColourRobot();

            % Logging the creation of the Aubo i5
            L.mlog = {L.DEBUG,'AuboI5','Aubo i5 object created within the workspace'};

            % Creating 2F-85 gripper and attaching it to the Aubo i5 end-effector
            self.UpdateToolTr; % Updating the end-effector transform property
            for gripperFinger = 1:2
                self.tool{1,gripperFinger} = TwoFingeredGripper(self.toolTr, gripperFinger, L);
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
            link(4) = Link([0   -0.1215   0        pi/2   0   0]);
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
            self.ellipsis = SerialLink(link,'name',[self.name,'_ellipsis']);
        end

        %% Updator for End Effector Transform (Tool Transform)
        function UpdateToolTr(self)
            % Updating the toolTr property of the robot
            % Used to update the pose of the gripper to the end-effector
            self.currentJointAngles = self.model.getpos(); % Getting the current joint angles of Aubo i5
            self.toolTr = self.model.fkine(self.currentJointAngles).T; % Updating toolTr property
        end

        %% Moving the Aubo i5 to a Desired Transform
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
                S = Rdot * Ra;
                linearVelocity = (1/deltaT) * deltaX; % Calculating the linear velocities in x-y-z
                angularVelocity = [S(3,2);S(1,3);S(2,1)]; % Calcualting roll-pitch-yaw angular velocity
                xdot = self.movementWeight*[linearVelocity; angularVelocity]; % Calculate end-effector matrix to reach next waypoint

                J = self.model.jacob0(qMatrix(i,:)); % Calculating the jacobian of the current joint state

                % Implementing Damped Least Squares
                manipulability(i,1) = sqrt(det(J*J')); % Calcualting the manipulabilty of the aubo i5
                disp(manipulability(i,1));
                if manipulability(i,1) < self.epsilon % Checking if manipulability is within threshold
                    lambda = (1 - (manipulability(i,1)/self.epsilon)^2) * self.maxLambda; % Damping Coefficient
                    
                else % If DLS isn't required
                    lambda = 0; % Damping Coefficient
                end

                invJ = inv(J'*J + lambda * eye(self.model.n))*J'; %#ok<MINV> % DLS inverse
                qdot(i,:) = (invJ * xdot)'; % Solving the RMRC equation
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT * qdot(i,:); % Updating next joint state based on joint velocities
            end
        end

        %% Creating the Ellipsis Around each Robot Link
        function UpdateEllipsis(self, q)
            % Creating an array of 4x4 transforms relating to robot links
            linkTransforms = zeros(4,4,(self.ellipsis.n)+1);                
            linkTransforms(:,:,1) = self.ellipsis.base; % Setting first transform as the base transform

            piFlag = 0;

            centre = [0 0 0;
                      -0.3 0 -0.125;
                      -0.3 0 -0.05;
                      0 0 0;
                      0 0 0;
                      0 0 0];
            
            mult =     [0.5 0.5 0.5;
                      0.66 0.66 0.66;
                      0.66 0.66 0.66;
                      0.5 0.5 0.5;
                      0.4 0.4 0.4;
                      0.125 0.125 0.125];
            
            % Getting the link data of the robot links
            links = self.ellipsis.links;

            for i = 1:length(links)
                L = links(1,i);
                
                current_transform = linkTransforms(:,:, i);
                
                current_transform = current_transform * trotz(q(1,i) + L.offset) * ...
                transl(0,0, L.d) * transl(L.a,0,0) * trotx(L.alpha);
                linkTransforms(:,:,i + 1) = current_transform;
            end

            for i = 1:length(links)

                A = 0.001;
                D = 0.001;

                if(i >= 2)

                    if(links(i-1).alpha == pi/2)
                        piFlag = ~piFlag;
                    end

                    if(piFlag)
                        A = A + links(i).d;
                        D = D + links(i).a;
                    else
                        A = A + links(i).a;
                        D = D + links(i).d;
                    end
                else
                     A = A + links(i).a;
                     D = D + links(i).d;
                end

                if (D > 0.001)
                    radii = [D, 0.2, 0.2];
                else
                     radii = [A, 0.2, 0.2];
                end
                

                [X, Y, Z] = ellipsoid(centre(i,1), centre(i,2), centre(i,3), radii(1), radii(2), radii(3));

                self.ellipsis.points{i+1} = [X(:)*mult(i,1),Y(:)*mult(i,2),Z(:)*mult(i,2)];
                self.ellipsis.faces{i+1} = delaunay(self.ellipsis.points{i+1}); 

            end

            centre = [0,0,0];
            radii = [0.1,0.1,0.1];

            [X, Y, Z] = ellipsoid(centre(1), centre(2), centre(3), radii(1), radii(2), radii(3));

            self.ellipsis.points{1} = [X(:),Y(:),Z(:)];
            self.ellipsis.faces{1} = delaunay(self.ellipsis.points{1}); 

            self.ellipsis.plot3d(self.model.getpos());
        end

        %% Checking if a Collision is Occurring with a Model
        function isCollision = CheckCollisions(self, jointAngles, model)
            isCollision = false; % Setting default return condition to state no present collisions
            
            % Creating an array of 4x4 transforms relating to robot links
            linkTransforms = zeros(4,4,(self.ellipsis.n)+1);                
            linkTransforms(:,:,1) = self.ellipsis.base; % Setting first transform as the base transform

            % Getting the link data of the robot links
            linkData = self.ellipsis.links;

            % Getting the points of the model that need to be checked
            points = [model.points{1,2}(:,1), model.points{1,2}(:,2), model.points{1,2}(:,3)];

            % For loop to get the remaining link transforms
            for i = 1:self.ellipsis.n
                % Calculating link transforms via link data
                linkTransforms(:,:,i+1) = linkTransforms(:,:,i) * trotz(jointAngles(i)) * transl(0,0,linkData(i).d) ...
                * transl(linkData(i).a,0,0) * trotx(linkData(i).alpha);
            end

            % Looping through each ellipsoid to check for collisions
            for i = 1:size(linkTransforms, 3)
                % Updating the points of the model relative to the links on the robot
                modelPointsAndOnes = (inv(linkTransforms(:,:,i)) * [points, ones(size(points,1),1)]')'; %#ok<MINV>
                updatedModelPoints = modelPointsAndOnes(:,1:3); % Getting the relative x-y-z points
    
                % Checking the algerbraic distance of these points
                algerbraicDist = LabAssessment2.GetAlgebraicDist(updatedModelPoints, self.linkCentres, self.linkRadii);
                    
                % Checking if the model is within the light curtain (i.e. there
                % is an algerbraic distance of < 1 with any of the above points
                if(find(algerbraicDist < 1) > 0)
                    isCollision = true; % Object has been detected within the light curtain
                    return; % Returning on first detection of object within the light curtain
                end
            end
        end

        %% Getter for the Algerbraic Distance between Objects and Light Curtain
        function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)
            algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
                  + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
                  + ((points(:,3)-centerPoint(3))/radii(3)).^2;
        end

    end
end
