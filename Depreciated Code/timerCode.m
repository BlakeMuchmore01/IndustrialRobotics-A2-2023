% Creating listeners connected to the GUI and its related callbacks
            addlistener(guiWindow,'StartButtonPressed', @(src,event) LabAssessment2.StartButtonPressed(src,event));
            addlistener(guiWindow,'EmergencyButtonPressed', @(src,event) LabAssessment2.EmergencyButtonPressed(src,event));
            addlistener(guiWindow,'DemonstrationModeChanged', @(src,event) LabAssessment2.DemonstrationModeChanged(src,event));
            addlistener(guiWindow,'BlackjackButtonRequest', @(src,event) LabAssessment2.BlackjackButtonRequest(src,event));
            addlistener(guiWindow,'CartesianCoordSent', @(src,event) LabAssessment2.CartesianCoordSent(src,event));
            addlistener(guiWindow,'CartesianCoordCancelled', @(src,event) LabAssessment2.CartesianCoordCancelled(src,event));
            addlistener(guiWindow,'JointMovementSent', @(src,event) LabAssessment2.JointMovementSent(src,event));

            % Using a timer to create an idler to wait to recieve incoming event requests from the GUI
            timerObj = timer('TimerFcn',@(~, ~) LabAssessment2.idleFunction, 'Period', 1, 'BusyMode', 'drop', 'ExecutionMode', 'fixedRate');
            start(timerObj); % Starting the idleFunction
            
            % Wait used to check if end demonstration functionality is called
            % while ~guiWindow.endDemonstration
            %     pause(0.1);
            %     disp('here');
            %     drawnow;
            % end