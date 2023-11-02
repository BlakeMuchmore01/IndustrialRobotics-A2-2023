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
    methods
        %% Constructor for Ardunino Object
        function self = Arduino()
            % Creating the serial port connection to the arduino
            self.ardObj = serialport(self.port, 9600);
        end
        
        %% Checking if the E-stop has been Pressed
        function eStopState = CheckButtonPressed(self)
            % Writing data to arduino uno to ask for pinState data
            write(self.ardObj,1,"uint8");

            % Reading the e-stop state that was provided
            data = readline(self.ardObj);
            eStopState = str2double(data); % Outputting arduino data
        end

        %% Forcing the pinState of E-stop to On
        function eStopState = EstopOn(self)
            % Writing to arduino to change the pinState to on
            write(self.ardObj,2,"uint8");

            % Reading the e-stop state that was provided
            data = readline(self.ardObj);
            eStopState = str2double(data); % Outputting arduino data
        end

        %% Forcing the pinState of the E-stop to Off
        function eStopState = EstopOff(self)
            % Writing to arduino to change the pinState to off
            write(self.ardObj,3,"uint8");

            % Reading the e-stop state that was provided
            data = readline(self.ardObj);
            eStopState = str2double(data); % Outputting arduino data
        end

        %% Testing Function for Writing
        function LED(self)
            % Writing data to arduino uno to then turn on LED
            write(self.ardObj,8,"uint8");
        end
    end
end