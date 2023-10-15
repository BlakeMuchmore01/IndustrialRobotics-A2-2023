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
                self.ardObj = serial('COM3', 'BaudRate', 9600);
                fopen(self.ardObj);
               
        end

        function stateChanged = CheckButtonPressed(self)

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

                data = fgetl(self.ardObj);
                disp(data);
        end

    end
end