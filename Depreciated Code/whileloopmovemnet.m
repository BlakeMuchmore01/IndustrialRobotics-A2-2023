            % While loop to check demonstration mode chosen by the GUI
            while (~guiWindow.EndDemonstration)
                % Switch statement to alter code functionality depending on state chosen previously
                switch guiWindow.simulationMode
                    % Functionality for the blackjack card distribution
                    case "Blackjack"
                        
                    % Functionality for the teaching/jogging
                    case "Teach"
                        % Looping functionality until end demonstration or mode is switched in the GUI
                        while (~guiWindow.EndDemonstration || guiWindow.simulationMode == "Teach") 
                            










                            drawnow; % Allowing the GUI properties to be updated within the loop
                        end
                end
                
                pause(0.1); % Pausing to reduce business of the while loop
                drawnow; % Allowing the GUI properties to be updated within the loop
            end