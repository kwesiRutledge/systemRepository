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

function test1_show(testCase)
	%Description:
	%	Shows the default position of the PusherSlider system.

	%% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	ps1 = PusherSlider();

	%% Algorithm

	figure;
	ps1.show();

	assert(true)

function test1_get_motion_cone_vectors(testCase)
	%Description:
	%	Tests the computation of the motion cone vectors gamma_t and gamma_b
	%	

	% Constants
	ps = PusherSlider();
	ps.ps_cof = 1;
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
	ps.ps_cof = 1;
	ps.s_width = 0.4; % c = 5
	ps.p_x = 2;
	ps.p_y = 2;

	% gamma_t_expected = 1;
	% gamma_b_expected = -1;
	%[gamma_t , gamma_b ] = ps.get_motion_cone_vectors();

	% Call Function
	mode_out1 = ps.identify_mode([2;0.5])

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
	mode_out2 = ps.identify_mode([2;3])

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
	mode_out3 = ps.identify_mode([2;-2.5])

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

