%% Real Dobot Manipulation Demo Code
% rosinit('192.168.27.1');
app = GUI;

%% Getting the current safety status of the robot
safetyStatusSubscriber = rossubscriber('/dobot_magician/safety_status');
pause(2); %Allow some time for MATLAB to start the subscriber
currentSafetyStatus = safetyStatusSubscriber.LatestMessage.Data

app.continuePressed = true;
pause;

while true
    endIndex = app.GetEndCardIndex();
    
    %% Getting the movement matrix to get to the card
    qMatrix = app.dobotMagician.GetCartesianMovement(app.playingCards.cardModels{endIndex}.base.T * transl(0,0,0.045));
    pause(0.5);

    % Looping through the qMatrix
    for i = 1:size(qMatrix, 1)
        % Checking if Emergency stop is pressed
        if app.arduino.CheckButtonPressed(app.arduino) == 1
            [safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
            safetyStateMsg.Data = 3;
            send(safetyStatePublisher,safetyStateMsg);
            app.RealEstopHit(1);
        else
            [safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
            safetyStateMsg.Data = 4;
            send(safetyStatePublisher,safetyStateMsg);
        end

        % Animating the real and sim dobot
        app.dobotMagician.model.animate(qMatrix(i,:));
        drawnow;
        
        jointTarget = [0.6944, 0.9513, 0.9047, 0]; % Remember that the Dobot has 4 joints by default.
        [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
        trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
        trajectoryPoint.Positions = jointTarget;
        targetJointTrajMsg.Points = trajectoryPoint;
        send(targetJointTrajPub,targetJointTrajMsg);
    end

    %% Setting suction cup
    % Turn on the tool
    [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');
    toolStateMsg.Data = [1]; %#ok<NBRAK2> % Send 1 for on and 0 for off 
    send(toolStatePub,toolStateMsg);

    %% Getting the movement matrix to get to the card
    initialPose = app.dobotMagician.model.fkine(app.dobotMagician.defaultRealQ).T;
    qMatrix = app.dobotMagician.GetCartesianMovement(initialPose);
    pause(0.5);

    % Looping through the qMatrix
    for i = 1:size(qMatrix, 1)
        % Checking if Emergency stop is pressed
        if app.arduino.CheckButtonPressed(app.arduino) == 1
            [safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
            safetyStateMsg.Data = 3;
            send(safetyStatePublisher,safetyStateMsg);
            app.RealEstopHit(1);
        else
            [safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
            safetyStateMsg.Data = 4;
            send(safetyStatePublisher,safetyStateMsg);
        end

        % Animating the real and sim dobot
        app.dobotMagician.model.animate(qMatrix(i,:));
        app.dobotMagician.UpdateToolTr;

        app.playingCards.cardModels{endIndex}.base = app.dobotMagician.toolTr * transl(0,0,-0.045);
        app.playingCards.cardModels{endIndex}.animate(0);
        drawnow;
        
        jointTarget = [0.0023, -0.0049, -0.1331, 0]; % Remember that the Dobot has 4 joints by default.
        [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
        trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
        trajectoryPoint.Positions = jointTarget;
        targetJointTrajMsg.Points = trajectoryPoint;
        send(targetJointTrajPub,targetJointTrajMsg);
    end

    app.NotifyCardDelt();
end
