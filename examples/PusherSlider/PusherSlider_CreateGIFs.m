%PusherSlider_CreateGIFs
%Description:
%	Creates gifs for my personal website of the PusherSlider System.

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

u = (1/2)*[0.3;0.0];

if ~exist('FileName')
	FileName = 'pusherslider_test1_f.gif';
end
vidObj = VideoWriter(FileName);

%% Simulating System Using ODE45
t_span = [0,2]
t_span2 = [0:0.01:2]
[ t_trajectory , x_trajectory ] = ode45(@(t,x) ps1.f(x,u) , t_span2 , ps1.x() )

%% Creating Video

% axis_limits = [	0, ...
% 				max(x_trajectory(:,1))+max(ps1.s_width,ps1.s_length), ...
% 				0, ...
% 				max(x_trajectory(:,2))+max(ps1.s_width,ps1.s_length) ]; 

axis_limits = [0,0.5,-0.3,0.4]

open(vidObj);

fig0 = figure;
for t = 1:length(t_trajectory)
	%Collect State from the input list.
	ps1.set_state(x_trajectory(t,:)');

	%Plot
	[h,c] = ps1.show();
	axis(axis_limits)
	
	% Save to gif file
	frame = getframe(fig0); 
	im = frame2im(frame); 
	[imind,cm] = rgb2ind(im,256); 
	% Write to the GIF File 
	if t == 1 
	  imwrite(imind,cm,FileName,'gif', 'Loopcount',inf); 
	elseif mod(t,10) == 0
	  imwrite(imind,cm,FileName,'gif','WriteMode','append'); %Save frames every ten instants.
	end 

	%Get Current frame and write it.
	currFrame = getframe;
	writeVideo(vidObj,currFrame);
	delete(h);
	delete(c);

	%Prepare for plot to be overwritten
	hold off; 

end

close(vidObj);

% Capture the plot as an image 
      