classdef Quadcopter
    %Quadcopter A representation of the quadcopter dynamical system.
    %   Quadcopter dynamics are given according to this:
    %   - `Trajectory Generation and Control for Quadrotors`
    %   [https://repository.upenn.edu/cgi/viewcontent.cgi?article=1705&context=edissertations
    %   ]
    %
    %Notes:
    %   - The state s is defined as:
    %
    %           [     x     ]
    %           [     y     ]
    %           [     z     ]
    %           [    phi    ]
    %           [   theta   ]
    %       s = [    psi    ]
    %           [   dot x   ]
    %           [   dot y   ]
    %           [   dot z   ]
    %           [  dot phi  ]
    %           [ dot theta ]
    %           [  dot psi  ]
    
    
    properties
        s;
    end
    
    methods
        function q_out = Quadcopter()
            %Quadcopter Construct an instance of this class
            %   Creates an instance of the quadcopter.
            s = [ 0 ; 0 ; 1 ; zeros(1,9) ]';
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

