classdef PusherSlider
    %PusherSlider A representation of the Pusher-Slider System
    %   An implementation of the model from `Feedback Control of the 
    %   Pusher-Slider System: A Story of Hybrid and Underactuated
    %   Contact Dynamics` by Francois Robert Hogan and Alberto Rodriguez.
    
    properties
        % State of the System
        s_x;        %x coordinate for the CoM of the Slider
        s_y;        %y coordinate for the CoM of the Slider
        s_theta;    %orientation of the CoM of the Slider w.r.t. Frame of reference a
        p_x;        %x position of the pusher w.r.t. Frame of reference b (attached to slider)
        p_y;        %y position of the pusher w.r.t. Frame of reference b (attached to slider)
        % Physical Parameters of System
        s_length;   %length of the slider
        s_width;    %width of the slider
        s_mass;     %mass of the slider
        ps_cof;     %Coefficient of friction between the pusher and the slider
        st_cof;     %Coefficient of friction between the slider and the table
        p_radius;   %Radius of the pusher
    end
    
    methods
        function ps = PusherSlider()
            %PusherSlider Construct an instance of this class
            %   Detailed explanation goes here
            
            %Define Default Physical Parameters
            ps.s_width = 0.09;
            ps.s_length = 0.09;
            ps.s_mass = 1.05; %kg
            ps.ps_cof = 0.3;
            ps.st_cof = 0.35;

            ps.p_radius = 0.01;
            
            % Define Initial State
            ps.s_x = 0.1;
            ps.s_y = 0.1;
            ps.s_theta = pi/6;
            ps.p_x = ps.s_width/2;
            ps.p_y = 0.02;
            
            
        end
        
        function show(varargin)
            %show Shows the system in a figure window.
            %   Uses the provided physical parameters to show the slider
            %   and pusher.
            
            % Input Processing
            % ================
            
            ps = varargin{1};
            
            % Constants
            % =========

            lw = 2.0;
            sliderColorChoice = 'blue';
            pusherColorChoice = 'magenta';


            % Creating Slider
            % ===============
            
            %Create lines.
            x_lb = -ps.s_width/2;
            x_ub =  ps.s_width/2;
            y_lb = -ps.s_length/2;
            y_ub =  ps.s_length/2;
            
            corners = [ x_lb, x_lb, x_ub, x_ub ;
                        y_lb, y_ub, y_ub, y_lb ];
            
            %Rotate and translate this.
            rot = [ cos(ps.s_theta), -sin(ps.s_theta) ;
                    sin(ps.s_theta), cos(ps.s_theta) ];
                
            rotated_corners = rot * corners;
            
            r_n_t_corners = rotated_corners + [ ps.s_x * ones(1,4); ps.s_y * ones(1,4) ]; %Rotated and translated corners
            
            % Plot
            hold on;
            for corner_idx = 1:size(r_n_t_corners,2)-1
                plot( ...
                    r_n_t_corners(1,corner_idx+[0:1]) , ...
                    r_n_t_corners(2,corner_idx+[0:1]) , ...
                    'LineWidth', lw , ...
                    'Color', sliderColorChoice ...
                )
            end
            plot( ...
                    r_n_t_corners(1,[4,1]) , ...
                    r_n_t_corners(2,[4,1]) , ...
                    'LineWidth', lw , ...
                    'Color', sliderColorChoice ...
                )
            hold off;

            % Create Pusher
            % =============

            % Create the circle
            circle_center = [ ps.s_x ; ps.s_y ] + rot*([ -ps.p_x ; ps.p_y ] + [ -ps.p_radius ; 0 ]);
            circle_ll = circle_center - ps.p_radius *ones(2,1);

            hold on;
            circ1 = rectangle(  'Position', [circle_ll(1),circle_ll(2),2*ps.p_radius*ones(1,2)],...
                                'Curvature',[1,1] , ...
                                'LineWidth', lw , ...
                                'FaceColor', pusherColorChoice );
            hold off;

            
        end
    end
end

