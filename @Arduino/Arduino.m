%% Establishing connection with Arduino and reading digital Pin for E-Stop
classdef Arduino < handle
    % Constant Properties
    properties (Constant)
        port = "COM3";
        board = "Uno";
        pin = "D13";   
    end

    % Non-constant Properties
    properties
        ardObj;
        pinState = 0;
    end

    %% Methods
    methods (Static)
        %% Constructor for Ardunino Object
        function self = Arduino()
                % self.ardObj = arduino(Arduino.port, Arduino.board);
                % self.pinState = readDigitalPin(self.ardObj, Arduino.pin);
                self.ardObj = serialport('COM3', 9600);
                % configureCallback(self.ardObj,"terminator",@CheckButtonPressed);
        end
        
        %% Checking if the E-stop has been Pressed
        function eStopState = CheckButtonPressed(self)

                % currentState = readDigitalPin(self.ardObj, self.pin);
                % 
                % if self.pinState ~= currentState
                % 
                %     self.pinState = currentState;
                %     stateChanged = true;
                %     disp("State Changed");
                % else
                %     stateChanged = false;
                %     disp("State Changed");
                % end
                
                % Writing data to arduino uno to then
                % receive e-stop state
                write(self.ardObj,1,"uint8");

                % Reading the e-stop state
                data = readline(self.ardObj);
                eStopState = str2num(data); % Outputting arduino data
        end

        function eStopState = EstopOn(self)

               write(self.ardObj,2,"uint8");
               data = readline(self.ardObj);
               eStopState = str2num(data);

        end

        function eStopState = EstopOff(self)

           write(self.ardObj,3,"uint8");
           data = readline(self.ardObj);
           eStopState = str2num(data);

        end

        %% Testing Function for Writing
        function LED(self)
            % Writing data to arduino uno to then turn on LED
            write(self.ardObj,8,"uint8");
        end
    end
end