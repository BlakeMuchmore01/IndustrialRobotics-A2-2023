        function HitSelected(auboI5, dobotMagician, guiWindow, cards, cardNum, player, logFile)
            % Getting the end card index within the cardModels array (i.e. one
            % that hasn't been delt prior)
            endCardIndex = guiWindow.GetEndCardIndex();
            nextCardIndex = endCardIndex-1; % Getting the index number for the next card

            % Getting the qMatrix to move the Aubo i5 to pick up the card
            % and getting the qMatrix to close the gripper
            qMatrixAubo = auboI5.GetCartesianMovement(LabAssessment2.handOffTransform);

            % Looping through the qMatrix to move the aubo to pick up the card and close the gripper 
            for i = 1:size(qMatrixAubo,1)
                %%%%%%%%%%%%%%%% CHECK COLLISION, LIGHT CURTAIN, ESTOP %%%%%%%%%%%%%%%
                
                % Animating the Aubo i5's movement and updating the gripper position
                auboI5.model.animate(qMatrixAubo(i,:));
                auboI5.UpdateToolTr(); % Updating the end-effector transform of the 

                LabAssessment2.LightCurtainCheck(guiWindow.hand.handModels{1}, guiWindow);

                % Updating the positions of the gripper fingers
                for gripperNum = 1:2
                    auboI5.tool{gripperNum}.UpdateGripperPosition(auboI5.toolTr, gripperNum);
                end
                drawnow;
            end
            
            % Closing the gripper of the aubo
            qMatrixGripper = auboI5.tool{1}.GetOpenCloseQMatrix();
            for i = 1:size(qMatrixGripper,1)
                %%%%%%%%%%%%%%%% CHECK COLLISION, LIGHT CURTAIN, ESTOP %%%%%%%%%%%%%%%
                LabAssessment2.LightCurtainCheck(guiWindow.hand.handModels{1}, guiWindow);
                % Looping through the qMatrix for both grippers
                for gripperNum = 1:2
                    auboI5.tool{gripperNum}.model.animate(qMatrixGripper(i,:));
                end
                drawnow; % Updating the plot
            end
            logFile.mlog = {logFile.DEBUG,'HitSelected','Aubo i5 collected the playing card'};

            % Getting the qMatrix to animate the Aubo i5 distributing the card to the player
            % Simultaneously move the dobot to start picking the next cards
            finalCardTransforms = cards.GetFinalCardTransforms(); % Getting the final poses of the cards
            qMatrixAubo = auboI5.GetCartesianMovement(finalCardTransforms(:,:,cardNum,player));
            qMatrixDobot = dobotMagician.GetCartesianMovement(cards.cardInitialTransforms{1,nextCardIndex} * transl(0,0,0.045));

            % Looping through the qMatrix's to distrubite the current card and pick up the next card
            for i = 1:size(qMatrixAubo, 1)
                %%%%%%%%%%%%%%%% CHECK COLLISION, LIGHT CURTAIN, ESTOP %%%%%%%%%%%%%%%
                auboI5.model.animate(qMatrixAubo(i,:)); % animating the model to the next pose in the qMatrix
                auboI5.UpdateToolTr(); % Updating the end-effector transform property


                LabAssessment2.LightCurtainCheck(guiWindow.hand.handModels{1}, guiWindow);

                % Updating the positions fo the gripper fingers
                for gripperNum = 1:2
                    auboI5.tool{gripperNum}.UpdateGripperPosition(auboI5.toolTr, gripperNum);
                end

                % Updating the position of the card alongside the gripper
                cards.cardModels{endCardIndex}.base = auboI5.toolTr * trotz(pi/2) * trotx(pi/2) * transl(0,0.2,-0.01);
                cards.cardModels{endCardIndex}.animate(0);

                dobotMagician.model.animate(qMatrixDobot(i,:)) % Animating the dobot
                dobotMagician.UpdateToolTr(); % Updating the end-effector transform property
                drawnow; % Updating the plot
            end
            logFile.mlog = {logFile.DEBUG,'HitSelected','Aubo i5 distributed the playing card'};
                
            guiWindow.NotifyCardDelt(); % Notifying that a card has been delt

            % Moving the aubo i5 to its original position and raise the dobot with the next card
            qMatrixAubo = auboI5.GetCartesianMovement(auboI5.model.fkine(auboI5.initialJointAngles).T);
            qMatrixGripper = auboI5.tool{1}.GetOpenCloseQMatrix(); % Getting qMatrix to open gripper
            qMatrixDobot = dobotMagician.GetCartesianMovement(dobotMagician.model.fkine(dobotMagician.defaultRealQ).T);

            % Looping through the qMatrix's
            for i = 1:size(qMatrixAubo)
                %%%%%%%%%%%%%%%% CHECK COLLISION, LIGHT CURTAIN, ESTOP %%%%%%%%%%%%%%%
                arduinoEstop = Arduino.CheckButtonPressed(app.arduino);

                auboI5.model.animate(qMatrixAubo(i,:)); % animating the model to the next pose in the qMatrix
                auboI5.UpdateToolTr(); % Updating the end-effector transform property
                
                LabAssessment2.LightCurtainCheck(guiWindow.hand.handModels{1}, guiWindow);


                % Updating the positions fo the gripper fingers
                for gripperNum = 1:2
                    auboI5.tool{gripperNum}.UpdateGripperPosition(auboI5.toolTr, gripperNum);
                    
                    % Opening the gripper within the first 100 steps
                    if i <= 100
                        auboI5.tool{gripperNum}.model.animate(qMatrixGripper(i,:));
                    end
                end

                dobotMagician.model.animate(qMatrixDobot(i,:)); % Animating the model to the next pose in the qMatrix
                dobotMagician.UpdateToolTr; % Updating the end-effector transform property
                
                % Updating the position of the card alongside the dobot
                cards.cardModels{nextCardIndex}.base = dobotMagician.toolTr * transl(0, 0, -0.045);
                cards.cardModels{nextCardIndex}.animate(0);
                drawnow; % Updating the plot
            end
        end