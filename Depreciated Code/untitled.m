% % Blackjack and teach functionality
            % while ~guiWindow.endDemonstration
            %     % Getting the robot that is being controlled by the user
            %     drawnow;
            %     switch guiWindow.RobotDropDown.Value
            %         case "Aubo I5"
            %             % Getting the updated joint angles and animating
            %             auboJointAngles = guiWindow.moveAuboDeg;
            %             auboI5.model.animate(deg2rad(auboJointAngles));
            %             drawnow;
            %         case "Dobot Magician"
            %             % Getting the updated joint angles and animating
            %             dobotJointAngles = guiWindow.moveDobotDeg;
            %             dobotMagician.model.animate(deg2rad(dobotJointAngles));
            %             drawnow;
            %     end
            %     drawnow;
            % end