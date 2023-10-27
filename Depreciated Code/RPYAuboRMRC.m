        %% Moving the Aubo i5 to a Desired Transform
        function qMatrix = GetCartesianMovementRMRC(self, coordinateTransform)
            deltaT = self.movementTime/self.movementSteps; % Calculating discrete time step

            % Allocating memory to data arrays
            manipulability = zeros(self.movementSteps, 1);      % Array of measure of manipulability
            qMatrix = zeros(self.movementSteps, self.model.n);  % Array of joint angle states
            qdot = zeros(self.movementSteps, self.model.n);     % Array of joint velocities
            theta = zeros(3, self.movementSteps);               % Array of end-effector angles
            trajectory = zeros(3, self.movementSteps);          % Array of x-y-z trajectory
            positionError = zeros(3,self.movementSteps); % For plotting trajectory error
            angleError = zeros(3,self.movementSteps);    % For plotting trajectory error

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
                theta(:,i) = rpy1; % rpy1*(1-s(i)) + s(i)*rpy2; % Creating the rotation trajectory
            end

            % Creating the transform for the first instance of the trajectory
            firstTr = [rpy2r(theta(1,1), theta(2,1), theta(3,1)) trajectory(:,1); zeros(1,3) 1];
            q0 = self.model.getpos(); % Getting the initial guess for the joint angles
            qMatrix(1,:) = self.model.ikcon(firstTr, q0); % Solving the qMatrix for the first waypoint

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
                deltaTheta = tr2rpy(Rd*Ra');

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

                positionError(:,i) = deltaX;             % For plotting
                angleError(:,i) = deltaTheta;            % For plotting
            end

            figure(2)
            subplot(2,1,1)
            plot(positionError'*1000,'LineWidth',1)
            refline(0,0)
            xlabel('Step')
            ylabel('Position Error (mm)')
            legend('X-Axis','Y-Axis','Z-Axis')
            
            subplot(2,1,2)
            plot(angleError','LineWidth',1)
            refline(0,0)
            xlabel('Step')
            ylabel('Angle Error (rad)')
            legend('Roll','Pitch','Yaw')

            figure(3)
            plot(manipulability,'k','LineWidth',1)
            refline(0,self.epsilon)
            title('Manipulability')
        end

                %% RMRC using Quaternions
        function qMatrix = GetQuaternionRMRC(self, coordinateTransform)
            deltaT = self.movementTime/self.movementSteps; % Calculating the discrete time step
        
            % Allocating memory to data arrays
            manipulability = zeros(self.movementSteps, 1);      % Array of measure of manipulability
            qMatrix = zeros(self.movementSteps, self.model.n);  % Array of joint angle states
            qdot = zeros(self.movementSteps, self.model.n);     % Array of joint velocities
            trajectory = zeros(3, self.movementSteps);          % Array of x-y-z trajectory
            quat = zeros(self.movementSteps, 4);                % Array of quaternions
            positionError = zeros(3,self.movementSteps); % For plotting trajectory error
            angleError = zeros(3,self.movementSteps);    % For plotting trajectory error
            aV = zeros(self.movementSteps, 3);
        
            % Getting the initial and final x-y-z coordinates
            initialTr = self.model.fkine(self.model.getpos()).T; % Getting the transform for the robot's current pose
            x1 = initialTr(1:3,4); % Extracting the x-y-z coordinates from the current pose transform
            x2 = coordinateTransform(1:3,4); % Extracting  the x-y-z coordiantes from the final pose transform
        
            % Getting the initial and final quaternions
            quaternion1 = rotm2quat(initialTr(1:3,1:3)); % Getting the initial quaternion
            quaternion2 = rotm2quat(coordinateTransform(1:3,1:3)); % Getting the final quaternion
        
            % Creating the movement trajectory
            s = lspb(0,1,self.movementSteps); % Creating interpolation scalar
            for i = 1:self.movementSteps
                trajectory(:,i) = x1*(1-s(i)) + s(i)*x2; % Creating the translation trajectory
        
                % Creating the rotational trajectory
                dotProduct = dot(quaternion1,quaternion2); % Calcualting the dot product of the two quaternions
        
                % If dot product is < 0, means quaternions are > 90deg apart
                % Changing sign of quaternion and dot product to calculate the shortest interpolation
                if dotProduct < 0
                    quaternion2 = - quaternion2;
                    dotProduct = -dotProduct;
                end
        
                % if dotProduct > 0.995
                %     quat(i, :) = (1 - s(i)) * quaternion1 + s(i) * quaternion2;
                % else
                %     theta_0 = acos(dotProduct);
                %     theta = theta_0 * s(i);
                %     sin_theta = sin(theta);
                %     sin_theta_0 = sin(theta_0);
                % 
                %     s0 = cos(theta) - dotProduct * sin_theta / sin_theta_0;
                %     s1 = sin_theta / sin_theta_0;
                % 
                %     quat(i, :) = (s0 * quaternion1) + (s1 * quaternion2);
                % end

                quat(i, :) = (1 - s(i)) * quaternion1 + s(i) * quaternion2;
            end
        
            % Creating the movement trajectory using RMRC
            for i = 1:self.movementSteps-1
                currentTr = self.model.fkine(qMatrix(i,:)).T; % Getting the forward transform at current joint states
                deltaX = trajectory(:,i+1) - currentTr(1:3,4); % Getting the position error from the next waypoint
        
                Rd = quat2rotm(quat(1,:)); % Getting the next rotation matrix
                Ra = currentTr(1:3,1:3); % Getting the current rotation matrix
        
                Rdot = (1/deltaT) * (Rd-Ra); % Calcualting the roll-pitch-yaw angular velocity rotation matrix
                S = Rdot * Ra';
                linearVelocity = (1/deltaT) * deltaX; % Calculating the linear velocities in x-y-z
                angularVelocity = [S(3,2);S(1,3);S(2,1)]; % Calcualting roll-pitch-yaw angular velocity
                xdot = self.movementWeight*[linearVelocity; angularVelocity]; % Calculate end-effector matrix to reach next waypoint
                deltaTheta = tr2rpy(Rd*Ra');

                aV(i,:) = angularVelocity';
        
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
        
                positionError(:,i) = deltaX;             % For plotting
                angleError(:,i) = deltaTheta;            % For plotting
            end
        
            figure(2)
            subplot(2,1,1)
            plot(positionError'*1000,'LineWidth',1)
            refline(0,0)
            xlabel('Step')
            ylabel('Position Error (mm)')
            legend('X-Axis','Y-Axis','Z-Axis')
            
            subplot(2,1,2)
            plot(angleError','LineWidth',1)
            refline(0,0)
            xlabel('Step')
            ylabel('Angle Error (rad)')
            legend('Roll','Pitch','Yaw')
        
            figure(3)
            plot(manipulability,'k','LineWidth',1)
            refline(0,self.epsilon)
            title('Manipulability')
        end