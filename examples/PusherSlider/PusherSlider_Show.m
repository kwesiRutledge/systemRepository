%PusherSlider_Show
%Description:
%	Tests that the f() function properly chooses the correct dynamics when given
%	an input.

%%%%%%%%%%%
%% Setup %%
%%%%%%%%%%%

if exist('PusherSlider') == 0
    %If the class does not exist on the path,
    %then add the systems directory to the path.
    addpath(genpath('../../systems'));
end

clear all;
close all;
clc;

%%%%%%%%%%%%%%%
%% Constants %%
%%%%%%%%%%%%%%%

ps1 = PusherSlider();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Show Initial Configuration %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure;
ps1.show();

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Modify State Slightly and Show %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ps1.set_state( ps1.x() - [ 0;0;0;0.03 ] )
figure;
ps1.show();