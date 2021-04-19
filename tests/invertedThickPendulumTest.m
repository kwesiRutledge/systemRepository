%test_InvertedThickPendulum.m
%Description:
%	Tests the InvertedThickPendulum class.

function tests = test_InvertedThickPendulum
	%disp(localfunctions)
	tests = functiontests(localfunctions);

function include_relevant_libraries()
	%Description:
	%	Attempts to add the relevant libraries/toolboxes to the path.

	%% Constants

    if exist('InvertedThickPendulum') == 0
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
	itp1 = InvertedThickPendulum();

	%% Algorithm
	assert( all(itp1.x == [0.1;0.05]) )

function test1_Show(testCase)
	%Description:
	%	Tests constructor

    %% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	itp1 = InvertedThickPendulum();
	itp1.x(1) = -1.1;

	%% Algorithm
	itp1.Show()


