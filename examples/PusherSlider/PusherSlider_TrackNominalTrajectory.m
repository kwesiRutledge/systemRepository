%PusherSlider_TrackNominalTrajectory
%Description:
%	Tests the ability to design an MPC controller that accurately tracks a given trajectory of the state.

%%%%%%%%%%%
%% Setup %%
%%%%%%%%%%%

clear all;
close all;
clc;

if exist('PusherSlider') == 0
    %If the class does not exist on the path,
    %then add the systems directory to the path.
    addpath(genpath('../../systems'));
end

%%%%%%%%%%%%%%%
%% Constants %%
%%%%%%%%%%%%%%%

ps1 = PusherSlider();

%%%%%%%%%%%%%%%%%%%%%%%
%% Create Trajectory %%
%%%%%%%%%%%%%%%%%%%%%%%
dt = 0.10;
t = [0:dt:7];

%% Creating s_y trajectory

v_x = 0.03;	%Assume constant lateral velocity
t_peak = 2;
t_end_curve = t_peak*1.5;
t_smooth = 4;

peak_val = t_peak * v_x;
end_curv_val = peak_val*sin((t_end_curve/t_peak)*pi/2);
smooth_val = t_peak * v_x / 4 ;

s_y_star = [];

for k = t
	if k <= t_end_curve
		s_y_star = [ s_y_star ; peak_val*sin((k/t_peak)*pi/2) ];
	elseif (t_end_curve < k) && (k <= t_smooth)
		s_y_star = [ s_y_star ; end_curv_val - (-smooth_val + end_curv_val ) / ( t_smooth - t_end_curve ) * (k - t_end_curve) ];
	else
		s_y_star = [ s_y_star ; smooth_val*exp(-(k-t_smooth)) ];
	end
end

figure;
plot(t,s_y_star)

%% Creating s_theta trajectory

s_theta_star = [];

% No need to compute derivatives and tangent vectors algorithmically, we can do it by hand.
for k = t
	if k <= t_end_curve
		s_theta_star = [ s_theta_star ; atan2( (pi/(2*t_peak))*peak_val*cos((k/t_peak)*pi/2), v_x ) ];
	elseif (t_end_curve < k) && (k <= t_smooth)
		s_theta_star = [ s_theta_star ; atan2( - (-smooth_val + end_curv_val ) / ( t_smooth - t_end_curve ) , v_x) ];
	else
		s_theta_star = [ s_theta_star ; atan2( -smooth_val*exp(-(k-t_smooth)) , v_x ) ];
	end
end

figure;
subplot(2,1,1)
plot(t,s_y_star)
subplot(2,1,2)
plot(t,s_theta_star)

%% Creating s_x trajectory

s_x_star = t'*v_x;

%% Creating p_y trajectory

p_y_star = [];
for k = t
	if k <= t_end_curve
		p_y_star = [ p_y_star ; ps1.p_y -  2*ps1.p_y*(k)/(t_end_curve)  ];
	elseif (t_end_curve < k) && (k <= t_smooth)
		p_y_star = [ p_y_star ; -ps1.p_y ];
	else
		p_y_star = [ p_y_star ; -ps1.p_y + ps1.p_y*(k-t_smooth)/(max(t)-t_smooth) ];
	end
end

x_star = [ s_x_star' ; s_y_star' ; s_theta_star' ; p_y_star' ];

%% Plot Trajectory

figure;

subplot(4,1,1)
plot(t,s_x_star)
xlabel('Time Index')
ylabel('$s_x$','Interpreter','latex')

subplot(4,1,2)
plot(t,s_y_star)
xlabel('Time Index')
ylabel('$s_y$','Interpreter','latex')

subplot(4,1,3)
plot(t,s_theta_star)
xlabel('Time Index')
ylabel('$s_{\theta}$','Interpreter','latex')

subplot(4,1,4)
plot(t,p_y_star)
xlabel('Time Index')
ylabel('$p_y$','Interpreter','latex')

figure;
plot(s_x_star,s_y_star)

%% Creating v_n trajectory

v_n_star = v_x * ones(size(t'));

v_t_star = [];
for k = t
	if k <= t_end_curve
		v_t_star = [ v_t_star ; -2*ps1.p_y/(t_end_curve)  ];
	elseif (t_end_curve < k) && (k <= t_smooth)
		v_t_star = [ v_t_star ; 0 ];
	else
		v_t_star = [ v_t_star ; ps1.p_y/(max(t)-t_smooth) ];
	end
end

figure;
subplot(2,1,1)
plot(t,v_n_star)
subplot(2,1,2)
plot(t,v_t_star)

u_star = [v_n_star';v_t_star'];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Simulate Using MPC Controller %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ps1.set_state(x_star(:,1));
TimeHorizon = 10;

axis_limits = [-0.1,0.4,-0.25,0.25];

if ~exist('FileName')
	FileName = 'pusherslider_hybridmpc_vid1.mp4';
end
vidObj = VideoWriter(FileName,'MPEG-4');
vidObj.FrameRate = (1/dt);
open(vidObj);

figure;
for t_index = 1:(length(t)-TimeHorizon+1)

	disp('Current State before LinearizedMPC')
	disp(ps1.x())

	[ u_bar_star , x_bar_star , opt_data ] = ps1.LinearizedMPC( ...
		ps1.x() , TimeHorizon , x_star(:,[t_index:t_index+TimeHorizon-1]) , u_star(:,[t_index:t_index+TimeHorizon-1]) , dt , ...
		'Q', 10*eye(4) , 'verbosity' , 0 );

	u = u_bar_star(:,1) + u_star(:,t_index);

	disp('Current State Before ode45')
	disp(ps1.x())

	[ t_trajectory , x_trajectory ] = ode45(@(t,x) ps1.f(x,u) , [0,dt] , ps1.x() );

	ps1.set_state(x_trajectory(end,:)');

	disp('Current State After ode45')
	disp(ps1.x())

	%Plot
	plot(s_x_star,s_y_star)
	hold on;
	
	x_temp = ps1.x();
	scatter(x_temp(1),x_temp(2))
	[h,c] = ps1.show();

	axis(axis_limits)
	
	%Get Current frame and write it.
	currFrame = getframe;
	writeVideo(vidObj,currFrame);
	delete(h);
	delete(c);

	%Prepare for plot to be overwritten
	hold off; 

	disp(['Completed t_' num2str(t_index) ' = ' num2str(t(t_index)) ])

end

close(vidObj)