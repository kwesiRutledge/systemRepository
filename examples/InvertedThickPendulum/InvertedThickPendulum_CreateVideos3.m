%InvertedThickPendulum_CreateVideos3.m
%Description:
%	Creating video to visualize difference in the dynamics when
%	there is friction in the rotational joint. This should cause the
%	pendulum to eventually settle.



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
itp.CoMx_rel = -0.25;
itp.mu_rot = 0.1;
%itp.CoMx_rel = 0;

if ~exist('FileName')
	FileName = 'invertedthickpendulum_test1_f.mp4';
end
vidObj = VideoWriter(FileName,'MPEG-4');

%%%%%%%%%%%%%%%%%%%%%%%%
%% Start Simulation 2 %%
%%%%%%%%%%%%%%%%%%%%%%%%

%% Simulating System Using ODE45
t_span = [0:0.01:2.5]
[ t_trajectory , x_trajectory ] = ode45(@(t,x) itp.f(x,0) , t_span , itp.x )

%% Creating Video

axis_limits = [	-itp.Length - 0.5, ...
				itp.Length+0.5, ...
				-itp.Length - 0.5, ...
				itp.Length + 0.5 ]; 

open(vidObj);

figure;
for t = 1:length(t_trajectory)
	%Collect State from the input list.
	itp.x = x_trajectory(t,:)';

	%Plot
	h = itp.Show();
	axis(axis_limits)
	
	%Get Current frame and write it.
	currFrame = getframe;
	writeVideo(vidObj,currFrame);
	delete(h);

	%Prepare for plot to be overwritten
	hold off; 

end

close(vidObj);