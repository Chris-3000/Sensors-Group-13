classdef DobotMagician < RobotBaseClass
    %% DobotMagician
    % This class is based on the DobotMagician. 
    % URL: https://en.dobot.cn/products/education/magician.html
    % 
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access =public)   
        plyFileNameStem = 'DobotMagician';

        %> defaultRealQ 
        defaultRealQ  = [0,pi/4,pi/4,0,0];
    end

    methods (Access = public) 
%% Constructor 
        function self = DobotMagician(baseTr)
			self.CreateModel();
            if nargin == 1			
				self.model.base = self.model.base.T * baseTr;
            end            
            self.homeQ = self.RealQToModelQ(self.defaultRealQ);
            self.PlotAndColourRobot();
        end

%% CreateModel
        function CreateModel(self)       
            link(1) = Link('d',0.103+0.0362,    'a',0,      'alpha',-pi/2,  'offset',0, 'qlim',[deg2rad(-135),deg2rad(135)]);
            link(2) = Link('d',0,        'a',0.135,  'alpha',0,      'offset',-pi/2, 'qlim',[deg2rad(5),deg2rad(80)]);
            link(3) = Link('d',0,        'a',0.147,  'alpha',0,      'offset',0, 'qlim',[deg2rad(-5),deg2rad(85)]);
            link(4) = Link('d',0,        'a',0.07,      'alpha',pi/2,  'offset',-pi/2, 'qlim',[deg2rad(-180),deg2rad(180)]);
            % link(5) = Link('d',-0.05,      'a',0,      'alpha',0,      'offset',pi, 'qlim',[deg2rad(-85),deg2rad(85)]);

            self.model = SerialLink(link,'name',self.name);
        end   
    end
    
    methods(Hidden)
%% TestMoveJoints
        % Overriding the RobotBaseClass function, since it won't work properly
        % for this robot
        function TestMoveJoints(self)
            self.TestMoveDobotMagician();
        end

%% Test Move DobotMagician
    function TestMoveDobotMagician(self)
            qPath = jtraj(self.model.qlim(:,1)',self.model.qlim(:,2)',50);                       
            for i = 1:50                
                self.model.animate(self.RealQToModelQ(qPath(i,:)));
%                 hold on;
%                 trplot(self.model.fkine(self.RealQToModelQ(qPath(i,:))));
                pause(0.2);
            end
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