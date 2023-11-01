%% Establishing connection with Arduino and reading digital Pin for E-Stop
classdef Arduino < handle
    % Constant Properties
    properties (Constant)
        port = "COM4"; % Port that the arduino is connected to on laptop
        board = "Uno"; % Arduino board type
        pin = "D13"; % Pin that arduino is reading from
    end

    % Non-constant Properties
    properties
        ardObj; % Property to store the serial port connection
        pinState = 0; % State of the e-stop
    end

    %% Methods
    methods (Static)
        %% Constructor for Ardunino Object
        function self = Arduino()
                % self.ardObj = arduino(Arduino.port, Arduino.board);
                % self.pinState = readDigitalPin(self.ardObj, Arduino.pin);
                self.ardObj = serialport('COM4', 9600);
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
                
                % Writing data to arduino uno to recieve e-stop data back
                write(self.ardObj,1,"uint8");

                % Reading the e-stop state that was provided
                data = readline(self.ardObj);
                eStopState = str2double(data); % Outputting arduino data
        end

        function eStopState = EstopOn(self)

               write(self.ardObj,2,"uint8");
               data = readline(self.ardObj);
               eStopState = str2double(data);

        end

        function eStopState = EstopOff(self)

           write(self.ardObj,3,"uint8");
           data = readline(self.ardObj);
           eStopState = str2double(data);

        end

        %% Testing Function for Writing
        function LED(self)
            % Writing data to arduino uno to then turn on LED
            write(self.ardObj,8,"uint8");
        end
    end
end