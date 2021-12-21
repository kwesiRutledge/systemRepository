%InvertedThickPendulum_PlottingLinearizedDynamics1.m
%Description:
%	Analyzing the behavior of the linearized dynamics about the initial state for the ITP.

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

tspan1 = [0:0.01:10];
x0 = [0.05;0.02];

%%%%%%%%%%%%%%%%%%%%%%%%
%% Start Simulation 1 %%
%%%%%%%%%%%%%%%%%%%%%%%%

%% Simulating System Using ODE45
itp.x = x0;
[ t_trajectory , x_trajectory ] = ode45(@(t,x) itp.f(x,0) , tspan1 , itp.x );

figure;
plot(t_trajectory,x_trajectory)
legend('$\theta$','$\dot{\theta}$','Interpreter','latex')
title('Nonlinear ITP With Zero Input From Default Initial Condition')

figure;
subplot(2,1,1)
plot(t_trajectory,x_trajectory(:,1))
xlabel('$t$','Interpreter','latex')
ylabel('$\theta$','Interpreter','latex')

subplot(2,1,2)
plot(t_trajectory,x_trajectory(:,2))
xlabel('$t$','interpreter','latex')
ylabel('$\dot{\theta}$','interpreter','LaTeX')

disp('Simulation Looks Good With Damping!')

%%%%%%%%%%%%%%%%%%%%%%%%
%% Start Simulation 2 %%
%%%%%%%%%%%%%%%%%%%%%%%%

tspan2 = [0:0.01:0.3];
itp.x = x0; %Reset system to initial condition.

[A1,B1] = itp.LinearizedContinuousDynamicsAbout( itp.x , 0 );

%Simulate Linearized System Starting from x0
[ t_trajectory2 , x_trajectory2 ] = ode45(@(t,x) A1 * x + B1 * 0 + itp.f(x0,0) , tspan2 , itp.x );

figure;
plot(t_trajectory2,x_trajectory2)
legend('$\theta$','$\dot{\theta}$','Interpreter','latex')
title('Linearized ITP With Zero Input From Default Initial Condition')

figure;
subplot(2,1,1)
hold on;
plot(t_trajectory2,x_trajectory([1:length(tspan2)],1))
plot(t_trajectory2,x_trajectory2(:,1))
xlabel('$t$','Interpreter','latex')
ylabel('$\theta$','Interpreter','latex')
title('Comparison of Nonlinear and Linearized System ($\theta$)','Interpreter','latex')

subplot(2,1,2)
hold on;
plot(t_trajectory2,x_trajectory([1:length(tspan2)],2))
plot(t_trajectory2,x_trajectory2(:,2))
xlabel('$t$','interpreter','latex')
ylabel('$\dot{\theta}$','interpreter','LaTeX')
title('Comparison of Nonlinear and Linearized System ($\dot{\theta}$)','Interpreter','latex')
