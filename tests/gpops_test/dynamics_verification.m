%% --------------------- %
% Test 2-d induction coupler dynamics for gpops %

sys_dynamics = @planarDynamics;
% ------------------------------------------------------------- %
%               Set up auxiliary data for problem               %
% ------------------------------------------------------------- %
load('coupler_splines.mat');
auxdata.fx = f_x_spline; % x force spline from coupler
auxdata.fy = f_y_spline; % y force spline from coupler
auxdata.r = 5; %sphere radius
auxdata.c = [0;-5];
auxdata.gamma = 1;

d1 = 0.1*[1/2^.5,-1/2^.5,0];
d2 = 0.1*[-1/2^.5,-1/2^.5,0];
a1 = [0,0,1];
a2 = [0,0,1];

auxdata.a = [a1;a2];
auxdata.d = [d1;d2];
input.auxdata = auxdata;

% ------------------------------------------------------------- %
%               Set up Inputs for problem               %
% ------------------------------------------------------------- %
x = 0;
y = 0.1;
theta = 0;
vx = 0;
vy = 0;
omega = 0;
u = [-10, 10];

input.phase.state(:,1) = x;
input.phase.state(:,2) = y ;
input.phase.state(:,3) = theta;
input.phase.state(:,4) = vx;
input.phase.state(:,5) = vy;
input.phase.state(:,6) = omega;
input.phase.control = u;
% ------------------------------------------------------------- %
%              Find dynamics               %
% ------------------------------------------------------------- %
phaseout = sys_dynamics(input);
disp(phaseout.dynamics);
%[r,v] = drawInspector(auxdata.a,auxdata.d);

