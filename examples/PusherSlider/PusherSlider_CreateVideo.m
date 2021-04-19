%PusherSlider_CreateVideo
%Description:
%	Tests that the f() function properly chooses the correct dynamics when given
%	an input.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Include Relevant Libraries %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if exist('PusherSlider') == 0
    %If the class does not exist on the path,
    %then add the systems directory to the path.
    addpath(genpath('../../systems'));
end

%% Constants
ps1 = PusherSlider();
ps1.p_y = -0.02;

[t1,t2] = ps1.get_motion_cone_vectors()

u = [0.1;0.2];

if ~exist('FileName')
	FileName = 'pusherslider_test1_f.mp4';
end
vidObj = VideoWriter(FileName,'MPEG-4');

%% Simulating System Using ODE45
t_span = [0,2]
t_span2 = [0:0.01:2]
[ t_trajectory , x_trajectory ] = ode45(@(t,x) ps1.f(x,u) , t_span2 , ps1.x() )

%% Creating Video

axis_limits = [	0, ...
				max(x_trajectory(:,1))+max(ps1.s_width,ps1.s_length), ...
				0, ...
				max(x_trajectory(:,2))+max(ps1.s_width,ps1.s_length) ]; 

open(vidObj);

figure;
for t = 1:length(t_trajectory)
	%Collect State from the input list.
	ps1.set_state(x_trajectory(t,:)');

	%Plot
	[h,c] = ps1.show();
	axis(axis_limits)
	
	%Get Current frame and write it.
	currFrame = getframe;
	writeVideo(vidObj,currFrame);
	delete(h);
	delete(c);

	%Prepare for plot to be overwritten
	hold off; 

end

close(vidObj);