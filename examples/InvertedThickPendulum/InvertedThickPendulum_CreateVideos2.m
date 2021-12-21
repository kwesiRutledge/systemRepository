%InvertedThickPendulum_CreateVideos2.m
%Description:
%	Plotting stuff for research update.

clear all;
close all;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Include Relevant Libraries %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if exist('PusherSlider') == 0
    %If the class does not exist on the path,
    %then add the systems directory to the path.
    addpath(genpath('../../systems'));
end

%%%%%%%%%%%%%%%
%% Constants %%
%%%%%%%%%%%%%%%

itp = InvertedThickPendulum();
itp.CoMx_rel = -0.75;


FileName1 = 'itp2_test1_show1.mp4';
vidObj = VideoWriter(FileName1,'MPEG-4');

%%%%%%%%%%%%%%%%%%%%
%% Create Image 1 %%
%%%%%%%%%%%%%%%%%%%%

figure;
[all_elts,pend_elts,com_elt,state_elt] = itp.Show()
delete(com_elt)
saveas(gcf,'itp2_show0','epsc')

%%%%%%%%%%%%%%%%%%%%%%%%
%% Start Simulation 1 %%
%%%%%%%%%%%%%%%%%%%%%%%%

%% Simulating System Using ODE45
t_span = [0:0.01:1.5]
[ t_trajectory , x_trajectory ] = ode45(@(t,x) itp.f(x,0) , t_span , itp.x )

%% Creating Video

axis_limits = [	-itp.Length - 0.5, ...
				itp.Length + 0.5, ...
				-itp.Length - 0.5, ...
				itp.Length + 0.5 ]; 

open(vidObj);

figure;
for t = 1:length(t_trajectory)
	%Collect State from the input list.
	itp.x = x_trajectory(t,:)';

	%Plot
	[all_elts,pend_elts,com_elt,state_elt] = itp.Show();
	axis(axis_limits)
	
	%Get Current frame and write it.
	currFrame = getframe;
	writeVideo(vidObj,currFrame);
	delete([pend_elts,com_elt]);

	%Prepare for plot to be overwritten
	hold off; 

end

close(vidObj);

%%%%%%%%%%%%%%%%%%%%%%%%
%% Start Simulation 2 %%
%%%%%%%%%%%%%%%%%%%%%%%%

itp = InvertedThickPendulum();
itp.CoMx_rel = 0;

FileName2 = 'itp2_test1_show2.mp4';
vidObj2 = VideoWriter(FileName2,'MPEG-4');

%% Simulating System Using ODE45
t_span = [0:0.01:1.5]
[ t_trajectory , x_trajectory ] = ode45(@(t,x) itp.f(x,0) , t_span , itp.x )

%% Creating Video

axis_limits = [	-itp.Length - 0.5, ...
				itp.Length + 0.5, ...
				-itp.Length - 0.5, ...
				itp.Length + 0.5 ]; 

open(vidObj2);

figure;
for t = 1:length(t_trajectory)
	%Collect State from the input list.
	itp.x = x_trajectory(t,:)';

	%Plot
	[all_elts,pend_elts,com_elt,state_elt] = itp.Show();
	axis(axis_limits)
	
	%Get Current frame and write it.
	currFrame = getframe;
	writeVideo(vidObj2,currFrame);
	delete([pend_elts,com_elt]);

	%Prepare for plot to be overwritten
	hold off; 

end

close(vidObj2);
