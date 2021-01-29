classdef FixedHeightPlanarLIPM
    %PlanarLIPM Class Defining a Planar Linear Inverted Pendulum Model
    %   Description:
    %   
    %   Assumptions:
    %       In this model, the center of mass of the pendulum remains at
    %       the same height.
    %
    %   References:
    %       This model comes from the work:
    %
    %       Posa, Michael, et al. “Balancing and Step Recovery Capturability
    %       via Sums-of-Squares Optimization.” Robotics: Science and Systems
    %       XIII, July, 2017, Cambridge, Massachusetts, Robotics: Science 
    %       and Systems Foundation, 2017. Version: Author's final manuscript
    
    properties
        zbar_cm;    %Center of Mass Height
        r_foot;     %Radius of the Foot
        XDimension; %Dimension of the x variable (the state)
        UDimension; %Dimension of the u variable (the input)
    end
    
    methods
        function systemOut = FixedHeightPlanarLIPM(varargin)
            %FixedHeightPlanarLIPM Construct an instance of the
            %fixed-height LIPM class
            %   Description:
            %       Creates an object which contains the dynamics
            %       definition of the fixed-height Linear Inverted
            %       Pendulum.
            %
            %   Usage:
            %       fhLIPM = FixedHeightPlanarLIPM()
            
            % Input Processing %
            
            if nargin ~= 0
                error('Unexpected inputs to the constructor.')
            end
            
            % Constants
            g = 10; %Gravitational Constant
            
            % Default System Parameters %
            
            systemOut.zbar_cm = 1;      %1 meter
            systemOut.r_foot = 0.05;    %0.05 meters
            
            systemOut.XDimension = 2;
            systemOut.UDimension = 1;
            
        end
        
%         function dxdt = Dynamics(varargin)
%             %Dynamics Returns dynamics of the Fixed Height Planar LIPM
%             %   Description:
%             %       When there are no inputs given to this function,
%             %       it returns a function (i.e. the function that maps
%             %       the system's states to outputs a la ODE45)
%             
%             %Create System
%             lipm = varargin{1};
%             
%             if nargin == 1
%                 dxdt = ones(lipm.XDimension,1);
%                 return
%             end
%         end
        
        function A_out = A(lipm)
            A_out = [   0 ,                         1 ;
                        (g/(systemOut.zbar_cm)) ,   0 ];
        end

        function B_out = B(lipm)
            %Description:
            %
            %Usage:
            %   B = lipm.B()

            B = [ 0 ; (g/(systemOut.zbar_cm))*systemOut.r_foot ];
        end

        function dxdt = ODEForm(lipm, t,x,u)
            A = lipm.A();
            B = lipm.B();
            dxdt = A*x + B*u;
        end
    end
end

