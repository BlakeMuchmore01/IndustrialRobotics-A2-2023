%% Real Dobot Manipulation Demo Code
function RealRobotControlGripper
    %% Initialising subscribers and publishers
    safetyStatusSubscriber = rossubscriber('/dobot_magician/safety_status');
    endEffectorPoseSubscriber = rossubscriber('/dobot_magician/end_effector_poses');
    jointStateSubscriber = rossubscriber('/dobot_magician/joint_states');
    [safetyStatePublisher,safetyStateMsg] = rospublisher('/dobot_magician/target_safety_status');
    [targetJointTrajPub,targetJointTrajMsg] = rospublisher('/dobot_magician/target_joint_states');
    [targetEndEffectorPub,targetEndEffectorMsg] = rospublisher('/dobot_magician/target_end_effector_pose');
    [toolStatePub, toolStateMsg] = rospublisher('/dobot_magician/target_tool_state');

    %% Creating GUI and Arduino Objects
    guiWindow = GUI();
    arduino = Arduino();

    %% Creating Joint States to Achieve
    jointStateHigh = deg2rad([0 15 45 120 0]);
    jointStateLow = deg2rad([0 0 0 0 0]);
    endEffectorRotation = [0 0 0];

    qMatrixUp = jtraj(jointStateLow,jointStateHigh,100);
    qMatrixDown = jtraj(jointStateHigh,jointStateLow,100);

    %% Getting the Current Safety Status of the Dobot
    currentSafetyStatus = safetyStatusSubscriber.LatestMessage.Data;
    disp(currentSafetyStatus);

    %% Moving the Dobot to it's Starting Joint State
    trajectoryPoint = rosmessage("trajectory_msgs/JointTrajectoryPoint");
    trajectoryPoint.Positions = jointStateHigh;
    targetJointTrajMsg.Points = trajectoryPoint;
    send(targetJointTrajPub,targetJointTrajMsg);
    pause;

    %% Creating While Loop to loop through high and low positions
    while (~guiWindow.endDemonstration)
        % Looping for down movement
        for i = 1:size(qMatrixDown,1)
            % Checking if E-Stop has been hit by either Arduino or GUI
            if(arduino.CheckButtonPressed())
                % Making estop pressed property true
                guiWindow.eStopPressed = true;
                guiWindow.startPressed = false;
            end
            
            % Looping while loop if estop has been pressed or if the continue
            % button hasnt been pressed after an e-stop
            if (guiWindow.eStopPressed || ~guiWindow.startPressed)
                pause(0.1);
                drawnow;
                continue;
            end
    
            % If above stopping conditions are passed, move the robot
            endEffectorPosition = qMatrixDown(i,:);
    
            targetEndEffectorMsg.Position.X = endEffectorPosition(1);
            targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
            targetEndEffectorMsg.Position.Z = endEffectorPosition(3);
            
            qua = eul2quat(endEffectorRotation);
            targetEndEffectorMsg.Orientation.W = qua(1);
            targetEndEffectorMsg.Orientation.X = qua(2);
            targetEndEffectorMsg.Orientation.Y = qua(3);
            targetEndEffectorMsg.Orientation.Z = qua(4);
            send(targetEndEffectorPub,targetEndEffectorMsg);
            drawnow;
        end

        % Activating gripper
        toolStateMsg.Data = [1]; %#ok<NBRAK2> % Send 1 for on and 0 for off 
        send(toolStatePub,toolStateMsg);

        % Looping for up movement
        for i = 1:size(qMatrixUp,1)
            % Checking if E-Stop has been hit by either Arduino or GUI
            if(arduino.CheckButtonPressed())
                % Making estop pressed property true
                guiWindow.eStopPressed = true;
                guiWindow.startPressed = false;
            end
            
            % Looping while loop if estop has been pressed or if the continue
            % button hasnt been pressed after an e-stop
            if (guiWindow.eStopPressed || ~guiWindow.startPressed)
                pause(0.1);
                drawnow;
                continue;
            end
    
            % If above stopping conditions are passed, move the robot
            endEffectorPosition = qMatrixUp(i,:);
    
            targetEndEffectorMsg.Position.X = endEffectorPosition(1);
            targetEndEffectorMsg.Position.Y = endEffectorPosition(2);
            targetEndEffectorMsg.Position.Z = endEffectorPosition(3);
            
            qua = eul2quat(endEffectorRotation);
            targetEndEffectorMsg.Orientation.W = qua(1);
            targetEndEffectorMsg.Orientation.X = qua(2);
            targetEndEffectorMsg.Orientation.Y = qua(3);
            targetEndEffectorMsg.Orientation.Z = qua(4);
            send(targetEndEffectorPub,targetEndEffectorMsg);
            drawnow;
        end

        % Deactivating Gripper
        toolStateMsg.Data = [0]; %#ok<NBRAK2> % Send 1 for on and 0 for off 
        send(toolStatePub,toolStateMsg);
    end
end