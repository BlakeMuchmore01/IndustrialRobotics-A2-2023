%% Real Dobot Manipulation Demo Code
% rosinit('192.168.27.1');
app = GUI;

%% Getting the current safety status of the robot
safetyStatusSubscriber = rossubscriber('/dobot_magician/safety_status');
pause(2); %Allow some time for MATLAB to start the subscriber
currentSafetyStatus = safetyStatusSubscriber.LatestMessage.Data;

while true
    %% Getting the movement matrix to get to the card
    qMatrix = app.dobotMagician.GetCartesianMovement(app.playingCards.cardModels{end}.base.T * transl(0,0,0.045));

    % Looping through the qMatrix
    for i = 1:size(qMatrix, 1)
        % Checking if Emergency stop is pressed
        if app.arduino.CheckButtonPressed()
            app.EmergencyStopButton();
        end

        % Animating the real and sim dobot
        app.dobotMagician.model.animate(qMatrix(i,:));
        
        jointTarget = qMatrix(i,1:4); % Remember that the Dobot has 4 joints by default.
        [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
        trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
        trajectoryPoint.Positions = jointTarget;
        targetJointTrajMsg.Points = trajectoryPoint;
        send(targetJointTrajPub,targetJointTrajMsg);

        pause(0.1);
        drawnow;
    end

    %% Setting suction cup
    % Turn on the tool
    [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
    toolStateMsg.Data = [1]; %#ok<NBRAK2> % Send 1 for on and 0 for off 
    send(toolStatePub,toolStateMsg);

    %% Getting the movement matrix to get to the card
    initialPose = self.model.fkine(self.defaultRealQ).T;
    qMatrix = self.GetCartesianMovement(initialPose);

    % Looping through the qMatrix
    for i = 1:size(qMatrix, 1)
        % Checking if Emergency stop is pressed
        if app.arduino.CheckButtonPressed()
            app.EmergencyStopButton();
        end

        % Animating the real and sim dobot
        app.dobotMagician.model.animate(qMatrix(i,:));
        app.dobotMagician.UpdateToolTr;

        app.playingCards.cardModels{end}.base = app.dobotMagician.toolTr * transl(0,0,-0.045);
        app.playingCards.cardModels{end}.animate(0);
        
        jointTarget = qMatrix(i,1:4); % Remember that the Dobot has 4 joints by default.
        [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
        trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
        trajectoryPoint.Positions = jointTarget;
        targetJointTrajMsg.Points = trajectoryPoint;
        send(targetJointTrajPub,targetJointTrajMsg);

        pause(0.1);
        drawnow;
    end

    %% Setting suction cup
    % Turn on the tool
    [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
    toolStateMsg.Data = [0]; %#ok<NBRAK2> % Send 1 for on and 0 for off 
    send(toolStatePub,toolStateMsg);
end
