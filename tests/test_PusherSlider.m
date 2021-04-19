%test_PusherSlider.m
%Description:
%	Tests the PusherSlider class.

function tests = test_PusherSlider
	%disp(localfunctions)
	tests = functiontests(localfunctions);

function include_relevant_libraries()
	%Description:
	%	Attempts to add the relevant libraries/toolboxes to the path.

	%% Constants

    if exist('PusherSlider') == 0
        %If the class does not exist on the path,
        %then add the systems directory to the path.
        addpath(genpath('../systems'));
    end

function test1_constructor(testCase)
	%Description:
	%	Tests constructor

    %% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	ps1 = PusherSlider();

	%% Algorithm

	assert( (ps1.p_x == 0.045) && (ps1.p_y==0.02) )

function test2_constructor(testCase)
	%Description:
	%	Tests constructor which now defines inputs.

    %% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	ps1 = PusherSlider();

	%% Algorithm

	assert( (ps1.v_n == 0.01) && (ps1.v_t==0.03) )

function test1_get_motion_cone_vectors(testCase)
	%Description:
	%	Tests the computation of the motion cone vectors gamma_t and gamma_b
	%	

	% Constants
	ps = PusherSlider();
	ps.st_cof = 1;
	ps.s_width = 1; % c = 2
	ps.p_x = 3;
	ps.p_y = 5;

	gamma_t_expected = (4-15+1*3.^2)/(4+5.^2-1*3*5);
	gamma_b_expected = (-4-15-1*3.^2)/(4+5.^2+1*3*5);

	% Call Function
	[gamma_t , gamma_b ] = ps.get_motion_cone_vectors();

	assert( (gamma_t == gamma_t_expected) && (gamma_b == gamma_b_expected) )

function test1_identify_mode(testCase)
	%Description:
	%	Tests the simple identification of the proper mode when gamma_t and gamma_b are nice.
	%	In this case, the mode should be "Sticking"

	% Constants
	ps = PusherSlider();
	ps.st_cof = 1;
	ps.s_width = 0.4; % c = 5
	ps.p_x = 2;
	ps.p_y = 2;

	% gamma_t_expected = 1;
	% gamma_b_expected = -1;
	%[gamma_t , gamma_b ] = ps.get_motion_cone_vectors();

	% Call Function
	mode_out1 = ps.identify_mode([2;0.5]);

	assert(strcmp(mode_out1,'Sticking'))

function test2_identify_mode(testCase)
	%Description:
	%	Tests the simple identification of the proper mode when gamma_t and gamma_b are nice.
	%	In this case, the mode should be "SlidingUp"

	% Constants
	ps = PusherSlider();
	ps.ps_cof = 1;
	ps.s_width = 0.4; % c = 5
	ps.p_x = 2;
	ps.p_y = 2;

	% gamma_t_expected = 1;
	% gamma_b_expected = -1;
	%[gamma_t , gamma_b ] = ps.get_motion_cone_vectors();

	% Call Function
	mode_out2 = ps.identify_mode([2;3]);

	assert(strcmp(mode_out2,'SlidingUp'))

function test3_identify_mode(testCase)
	%Description:
	%	Tests the simple identification of the proper mode when gamma_t and gamma_b are nice.
	%	In this case, the mode should be "SlidingDown"

	% Constants
	ps = PusherSlider();
	ps.ps_cof = 1;
	ps.s_width = 0.4; % c = 5
	ps.p_x = 2;
	ps.p_y = 2;

	% gamma_t_expected = 1;
	% gamma_b_expected = -1;
	%[gamma_t , gamma_b ] = ps.get_motion_cone_vectors();

	% Call Function
	mode_out3 = ps.identify_mode([2;-2.5]);

	assert(strcmp(mode_out3,'SlidingDown'))

function test1_C(testCase)
	%Description:
	%	Tests the creation of the C rotation matrix for a known orientation.

	% Constants
	ps = PusherSlider();
	ps.s_theta = pi/6;

	assert( all(all(ps.C == [cos(pi/6), sin(pi/6); -sin(pi/6), cos(pi/6)] ) ) )

function test1_Q(testCase)
	%test1_Q
	%Description:
	%	Tests the creation of a simple Q matrix from the modes of the system

	% Constants
	ps = PusherSlider();
	ps.ps_cof = 1;
	ps.s_width = 0.4; % c = 5
	ps.p_x = 2;
	ps.p_y = 2;

	% Algorithm
	Q1 = ps.Q;

	Q_temp = (1/(25+8)) * [ 25+4, 4; 4, 25+4 ];

	assert(all(all( Q1 == Q_temp )))

function test1_f(testCase)
	%Description:
	%	Tests the f() function so that we know that the state of the pusher slider is not modified
	%	after evaluating f at a given point.

	%% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	ps1 = PusherSlider();

	x0 = ps1.get_state();
	u_prime = [1;3];
	x1 = x0 + [0.1;zeros(3,1)];

	B = ps1.f( x1 , u_prime );

	assert(all(ps1.get_state() == x0))


function test1_LinearizedSystemAbout(testCase)
	%test1_LinearizedAbout
	%Description:
	%	Tests that the LinearizedAbout() function properly chooses the correct dynamics when given
	%	an input.

	%% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	ps1 = PusherSlider();

	x = ps1.get_state();
	u = [0.1;0.2];

	[A,B] = ps1.LinearizedSystemAbout(x,u)

	% eval(A)
	eig(A)

function test1_u(testCase)
	%Description:
	%	Tests ability to extract the current value of the input u

    %% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	ps1 = PusherSlider();

	%% Algorithm

	assert( all(ps1.u() == [0.01 ; 0.03]) )

function test1_set_input(testCase)
	%Description:
	%	Verifies that set input can correctly change the default.

	%% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	ps1 = PusherSlider();
	u_prime = [1;3];

	%% Algorithm
	ps1.set_input(u_prime)

	assert( all(ps1.u() == u_prime) )

function test1_A1(testCase)
	%Description:
	%	Verifies that A1() function does not change the value of the state.

	%% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	ps1 = PusherSlider();

	x0 = ps1.get_state();
	u_prime = [1;3];
	x1 = x0 + [0.1;zeros(3,1)];

	A = ps1.A1( x1 , u_prime );

	assert(all(ps1.get_state() == x0))

function test1_A2(testCase)
	%Description:
	%	Verifies that A2() function does not change the value of the state.

	%% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	ps1 = PusherSlider();

	x0 = ps1.get_state();
	u_prime = [1;3];
	x1 = x0 + [0.1;zeros(3,1)];

	A = ps1.A2( x1 , u_prime );

	assert(all(ps1.get_state() == x0))

function test1_A3(testCase)
	%Description:
	%	Verifies that A3() function does not change the value of the state.

	%% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	ps1 = PusherSlider();

	x0 = ps1.get_state();
	u_prime = [1;3];
	x1 = x0 + [0.1;zeros(3,1)];

	A = ps1.A3( x1 , u_prime );

	assert(all(ps1.get_state() == x0))

function test1_B1(testCase)
	%Description:
	%	Verifies that B1() function does not change the value of the state.

	%% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	ps1 = PusherSlider();

	x0 = ps1.get_state();
	u_prime = [1;3];
	x1 = x0 + [0.1;zeros(3,1)];

	B1 = ps1.B1( x1 , u_prime );

	assert(all(ps1.get_state() == x0))

function test1_B2(testCase)
	%Description:
	%	Verifies that B2() function does not change the value of the state.

	%% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	ps1 = PusherSlider();

	x0 = ps1.get_state();
	u_prime = [1;3];
	x1 = x0 + [0.1;zeros(3,1)];

	B = ps1.B2( x1 , u_prime );

	assert(all(ps1.get_state() == x0))

function test1_B3(testCase)
	%Description:
	%	Verifies that B3() function does not change the value of the state.

	%% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	ps1 = PusherSlider();

	x0 = ps1.get_state();
	u_prime = [1;3];
	x1 = x0 + [0.1;zeros(3,1)];

	B = ps1.B3( x1 , u_prime );

	assert(all(ps1.get_state() == x0))


function test1_LinearizedMPC(testCase)
	%Description:
	%	Verifies that LinearizedMPC can accurately find when the input x0 is not of the proper dimension.

	%% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	ps1 = PusherSlider();
	u_prime = [1;3];

	%% Algorithm
	try
		ps1.LinearizedMPC(u_prime,1,2,3)
		assert(false)
	catch e
		disp(e.message)
		assert(strcmp(e.message,['Expected the input x0 to be four dimensional. Instead it has dimension 2.' ]))
	end

function test2_LinearizedMPC(testCase)
	%Description:
	%	Verifies that LinearizedMPC can accurately find when not enough inputs are given.

	%% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	ps1 = PusherSlider();
	u_prime = [1;3];

	%% Algorithm
	try
		ps1.LinearizedMPC(u_prime,1,2)
		assert(false)
	catch e
		disp(e.message)
		assert(strcmp(e.message,['Expected at least 5 arguments. Received 4.' ]))
	end

function test3_LinearizedMPC(testCase)
	%Description:
	%	Verifies that LinearizedMPC can accurately find when the input an improper flag is given.

	%% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	ps1 = PusherSlider();
	u_prime = [1;3];

	%% Algorithm
	try
		ps1.LinearizedMPC(u_prime,1,2,3,'Z',2)
		assert(false)
	catch e
		disp(e.message)
		assert(strcmp(e.message,['Unexpected extra input to LinearizedMPC: Z' ]))
	end

function test4_LinearizedMPC(testCase)
	%Description:
	%	Verifies that LinearizedMPC can accurately find when the input x0 is not of the proper dimension.

	%% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	ps1 = PusherSlider();
	u_prime = [1;3];

	%% Algorithm
	try
		ps1.LinearizedMPC(u_prime,1,2,3,'Z',2)
		assert(false)
	catch e
		disp(e.message)
		assert(strcmp(e.message,['Unexpected extra input to LinearizedMPC: Z' ]))
	end