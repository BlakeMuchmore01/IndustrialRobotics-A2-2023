%% Dobot Magician Robot
% URL: https://en.dobot.cn/products/education/magician.html

classdef DobotMagician < RobotBaseClass
    %% Robot Class Properties
    % Non-constant properties
    properties (Access = public)   
        plyFileNameStem = 'DobotMagician'; % Name stem used to find associated ply model files
    end

    % Constant properties
    properties (Access = public, Constant)
        defaultRealQ  = [0,pi/4,pi/4,0,0]; % Default joint angle
    end

    %% ...structors
    methods (Access = public) 
        %% Constructor 
        function self = DobotMagician(baseTr, L)
			% Setting the default base transform if not set within
            % constructor inputs
            if nargin < 2
                % Creating log file if not supplied (indicates it hasn't
                % been created yet)
                L = log4matlab('logFile.log');
                L.SetCommandWindowLevel(L.DEBUG);

                if nargin < 1
                    % Logging that the default base transform has been used
                    L.mlog = {L.DEBUG,'DobotMagician','Base transform not set. Default base transform used'};
                    baseTr = eye(4); % Setting base transform as default
                end
            end
            
            self.CreateModel(); % Creating the dobot magician D&H parameter model

            % Orientating the dobot magician within the workspace
            self.model.base = self.model.base.T * baseTr; % Updating the base of the robot to input base tr
            self.homeQ = self.RealQToModelQ(self.defaultRealQ); % Setting initial starting pose of robot
            self.PlotAndColourRobot(); % Plotting the dobot magician and associated ply models

            % Logging the creation of the Dobot Magician
            L.mlog = {L.DEBUG,'DobotMagician','Dobot Magician object created within the workspace'};
        end

        %% D&H Parameter Serial Link Creation
        function CreateModel(self)
            % D&H parameters for the Aubo i5 model
            % DH = [THETA D A ALPHA SIGMA OFFSET]
            % URL: https://en.dobot.cn/products/education/magician.html
            link(1) = Link([0    0.103+0.0362   0       -pi/2   0    0]);
            link(2) = Link([0    0              0.135    0      0   -pi/2]);
            link(3) = Link([0    0              0.147    0      0    0]);
            link(4) = Link([0    0              0.060    pi/2   0   -pi/2]);
            link(5) = Link([0   -0.05           0        0      0    pi]);

            % Qlims for each joint
            link(1).qlim = [-135 135]*pi/180;
            link(2).qlim = [5    80]*pi/180;
            link(3).qlim = [-5   85]*pi/180;
            link(4).qlim = [-180 180]*pi/180;
            link(5).qlim = [-85  85]*pi/180;

            % Creating the serial link object
            self.model = SerialLink(link,'name',self.name);
        end   
    end
    
    methods(Static)
        %% RealQToModelQ
        % Convert the real Q to the model Q
        function modelQ = RealQToModelQ(realQ)
            modelQ = realQ;
            modelQ(3) = DobotMagician.ComputeModelQ3GivenRealQ2and3( realQ(2), realQ(3) );
            modelQ(4) = pi - realQ(2) - modelQ(3);    
        end
        
        %% ModelQ3GivenRealQ2and3
        % Convert the real Q2 & Q3 into the model Q3
        function modelQ3 = ComputeModelQ3GivenRealQ2and3(realQ2,realQ3)
            modelQ3 = pi/2 - realQ2 + realQ3;
        end
        
        %% ModelQToRealQ
        % Convert the model Q to the real Q
        function realQ = ModelQToRealQ( modelQ )
            realQ = modelQ;
            realQ(3) = DobotMagician.ComputeRealQ3GivenModelQ2and3( modelQ(2), modelQ(3) );
        end
        
        %% RealQ3GivenModelQ2and3
        % Convert the model Q2 & Q3 into the real Q3
        function realQ3 = ComputeRealQ3GivenModelQ2and3( modelQ2, modelQ3 )
            realQ3 = modelQ3 - pi/2 + modelQ2;
        end
    end
end