%------------------- Induction Inspector Path --------------------%
%                                      %
%-----------------------------------------------------------------%

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

% --------------------------------------------------------------%
%           Set up bounds on state, control, and time           %
% --------------------------------------------------------------%
t0 = 0;
tf = 5;
t_err = 0.5;
x0       = 0;  xf       = 0;
y0       = 0.1;  yf       = 0.085;
theta0   = 0; thetaf   = 0;
vx0      = 0;    vxf      = 0;
vy0      = 0;    vyf      = 0;
omega0   = 0;    omegaf   = 0;

xmin     = -1; xmax     = 1;
ymin     = 0; ymax     = 3;
thetamin = -pi; thetamax = pi;
vxmin    = -2;  vxmax    = 2;
vymin    = -2;  vymax    = 2;
omegamin = -1;  omegamax = 1;

% The control is six-dimensional in this formulation.  
% The "real" controls are u1-u2 and u3-u4.  In order to
% use this formulation it is necessary to include the 
% control constraints ui>=0 (i=1,...4) along with the 
% path constraints u1+u2<=1 and u3+u4<=1.
u1Min = -10000; u1Max = 10000;
u2Min = -10000; u2Max = 10000;

bounds.phase.initialtime.lower = t0;
bounds.phase.initialtime.upper = t0;
bounds.phase.finaltime.lower   = tf;
bounds.phase.finaltime.upper   = tf;

bounds.phase.initialstate.lower = [x0,y0,theta0,vx0,vy0,omega0];
bounds.phase.initialstate.upper = [x0,y0,theta0,vx0,vy0,omega0];
bounds.phase.state.lower        = [xmin,ymin,thetamin,vxmin,vymin,omegamin];
bounds.phase.state.upper        = [xmax,ymax,thetamax,vxmax,vymax,omegamax];
bounds.phase.finalstate.lower   = [xf,yf,thetaf,vxf,vyf,omegaf];
bounds.phase.finalstate.upper   = [xf,yf,thetaf,vxf,vyf,omegaf];
bounds.phase.control.lower = [u1Min,u2Min];
bounds.phase.control.upper = [u1Max,u2Max];
%TODO look up what these represent
bounds.phase.integral.lower = 0;
bounds.phase.integral.upper = 1000000000;
bounds.phase.path.lower = [-10000];
bounds.phase.path.upper = [10000];

tGuess     = [t0; tf];
xGuess     = [x0; xf];
yGuess     = [y0; yf];
thetaGuess = [theta0; thetaf];
vxGuess    = [vx0; vxf];
vyGuess    = [vy0; vyf];
omegaGuess = [omega0; omegaf];
u1Guess    = [0; 0];
u2Guess    = [0; 0];

guess.phase.time = tGuess;
guess.phase.state = [xGuess,yGuess,thetaGuess,vxGuess,vyGuess,omegaGuess];
guess.phase.control = [u1Guess,u2Guess];
guess.phase.integral = 0;

mesh.method = 'hp-LiuRao';
mesh.tolerance = 1e-4; 
mesh.maxiteration = 40;
mesh.colpointsmin = 4;
mesh.colpointsmax = 10;
mesh.phase.colpoints = 4*ones(1,10);
mesh.phase.fraction = 0.1*ones(1,10);

% ----------------------------------------------------------------------- %
% Set up solver
% ----------------------------------------------------------------------- %
setup.name = 'Induction_Inspector';
setup.functions.continuous = @planarDynamics;
setup.functions.endpoint = @planarDynamicsEndpoint;
setup.auxdata = auxdata;
setup.bounds = bounds;
setup.guess = guess;
setup.mesh = mesh;
setup.nlp.solver = 'ipopt';
setup.nlp.ipoptoptions.maxiterations = 10;
setup.nlp.ipoptoptions.tolerance = 1e-4;
setup.displaylevel = 2;
setup.scales.method = 'automatic-bounds';

% ----------------------------------------------------------------------- %
% Solve problem and extract solution
% ----------------------------------------------------------------------- %
output = gpops2(setup);