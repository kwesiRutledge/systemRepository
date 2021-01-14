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
