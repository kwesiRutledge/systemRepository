%test_FixedHeightPlanarLIPM.m
%Description:
%	Tests the FixedHeightPlanarLIPM class.

function tests = test_FixedHeightPlanarLIPM
	%disp(localfunctions)
	tests = functiontests(localfunctions);

function include_relevant_libraries()
	%Description:
	%	Attempts to add the relevant libraries/toolboxes to the path.

	%% Constants

    if exist('FixedHeightPlanarLIPM') == 0
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
	lipm1 = FixedHeightPlanarLIPM();

	%% Algorithm

	assert( (lipm1.zbar_cm == 1) && (lipm1.r_foot == 0.05) )

function test2_constructor(testCase)
	%Description:
	%	Tests constructor's linear system matrices

    %% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	lipm1 = FixedHeightPlanarLIPM();

    A2 = [ 0 , 1 ; 10 , 0 ];
    B2 = [ 0 ; 0.5]; 
    
	%% Algorithm

	assert( all(all(lipm1.A == A2)) && all(lipm1.B == B2) )
