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

	assert( all(all(lipm1.A() == A2)) && all(lipm1.B() == B2) )

function test1_Ad(testCase)
	%Description:
	%	Tests that the discretized Ad matrix computes the proper value for Ad
	%	when compared to what c2d produces.

	%% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	lipm1 = FixedHeightPlanarLIPM();

    A1 = [ 0 , 1 ; 10 , 0 ];
    B1 = [ 0 ; 0.5]; 

    dt = 0.1;

    temp_sys = ss(A1,B1,eye(2),0);
    temp_dsys = c2d(temp_sys,dt);

    assert( all(all( temp_dsys.A == lipm1.Ad(dt) )) )

function test2_Ad(testCase)
	%Description:
	%	Tests that the discretized Ad matrix computes the proper value for Ad
	%	when compared to what matrix exponential finds.

	%% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	lipm1 = FixedHeightPlanarLIPM();

    A1 = [ 0 , 1 ; 10 , 0 ];
    B1 = [ 0 ; 0.5]; 

    dt = 0.1;

    assert( all(all( expm(A1*dt) == lipm1.Ad(dt) )) )

function test1_Bd(testCase)
	%Description:
	%	Tests that the discretized Bd matrix computes the proper value for Ad
	%	when compared to what c2d produces.

	%% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	lipm1 = FixedHeightPlanarLIPM();

    A1 = [ 0 , 1 ; 10 , 0 ];
    B1 = [ 0 ; 0.5]; 

    dt = 0.1;

    temp_sys = ss(A1,B1,eye(2),0);
    temp_dsys = c2d(temp_sys,dt);

    temp_diff = temp_dsys.B - lipm1.Bd(dt);


    assert( all(all( temp_diff < 10^(-10) )) )

function test2_Bd(testCase)
	%Description:
	%	Tests that the discretized Bd matrix computes the proper value for Ad
	%	when compared to what vector integral produces.

	%% Include Relevant Libraries
    include_relevant_libraries();
    
	%% Constants
	lipm1 = FixedHeightPlanarLIPM();

    A1 = [ 0 , 1 ; 10 , 0 ];
    B1 = [ 0 ; 0.5]; 

    dt = 0.1;

    Bd_expected = integral(@(t) expm(A1*t)*B1,0,dt, 'ArrayValued',true);

    assert( all(all( Bd_expected == lipm1.Bd(dt) )) )


