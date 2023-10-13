classdef Arduino < handle
    % Establishing connection with Arduino and reading digital Pin for
    % E-Stop

    properties (Constant)

        
        port = "COM4";
        board = "Uno";
        pin = "D13";
        
            
    end

    properties

        pinState = 0;

    end

    methods (Static)

        function self = InitiateArduino()
                
                self = arduino(Arduino.port, Arduino.board);
                self.pinState = readDigitalPin(self, self.pin);
               
        end

        function stateChanged = CheckButtonPressed(self)

                currentState = readDigitalPin(self, Arduino.pin);

                if self.pinState ~= currentState

                    self.pinState = currentState;
                    stateChanged = true;
                else
                    stateChanged = false;
                end
        end

    end
end