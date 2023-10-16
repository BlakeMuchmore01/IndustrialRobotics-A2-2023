classdef Arduino < handle
    % Establishing connection with Arduino and reading digital Pin for
    % E-Stop

    properties (Constant)

        port = "COM3";
        board = "Uno";
        pin = "D13";   

    end

    properties

        ardObj;
        pinState = 0;

    end

    methods

        function self = Arduino()
                
                % self.ardObj = arduino(Arduino.port, Arduino.board);
                % self.pinState = readDigitalPin(self.ardObj, Arduino.pin);
                self.ardObj = serialport('COM3', 9600);
                configureCallback(self.ardObj,"terminator",@CheckButtonPressed);

        end

        function CheckButtonPressed(self)

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

                write(self.ardObj,8,"uint8");

                data = readline(self.ardObj);
                disp(data);
        end

        function LED(self)


           write(self.ardObj,8,"uint8");

        end
    end
end