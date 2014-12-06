function output = dircolGpops(obj,options)
%------------------- Induction Inspector Path --------------------%
%                                      %
%-----------------------------------------------------------------%

% ------------------------------------------------------------- %
%               Set up auxiliary data for problem               %
% ------------------------------------------------------------- %

%Total time with spline evaluations 95 s
% load('coupler_splines.mat');
% auxdata.fx = f_x_spline; % x force spline from coupler
% auxdata.fy = f_y_spline; % y force spline from coupler
auxdata.r = obj.sphere_radius; %sphere radius
auxdata.c = obj.sphere_center;
auxdata.gamma = 1;


auxdata.a = obj.a;
auxdata.d = obj.d;

% --------------------------------------------------------------%
%           Set up bounds on state, control, and time           %
% --------------------------------------------------------------%
phi = pi/50; %angle to rotate around surface
t0 = 0;
tf = 30;
t_err = 10;
x_err = [0.001;0.001;0.001;0;0;0];
% % final conditions for rotating around surface
% x0       = 0;  xf       = auxdata.r*sin(phi);
% y0       = 0.1;  yf  = y0 + auxdata.r*cos(phi) - auxdata.r;
% theta0   = 0; thetaf   = -phi;
% %

x0       = 0;     xf      = 0.1;
y0       = 0.21;  yf        = 0.21;
theta0   = 0;     thetaf  = 0;  
vx0      = 0;    vxf      = 0;
vy0      = 0;    vyf      = 0;
omega0   = 0;    omegaf   = 0;

if isfield(options,'x0')
    x0 = options.x0(1);
    y0 = options.x0(2);
    theta0 = options.x0(3);
    vx0 = options.x0(4);
    vy0 = options.x0(5);
    omega0 = options.x0(6);
    
end
if isfield(options,'xf')
     xf = options.xf(1);
    yf = options.xf(2);
    thetaf = options.xf(3);
    vxf = options.xf(4);
    vyf = options.xf(5);
    omegaf = options.xf(6);
end

xmin     = -3; xmax     = 3;
ymin     = -3; ymax     = 3;
thetamin = -pi; thetamax = pi;
vxmin    = -2;  vxmax    = 2;
vymin    = -2;  vymax    = 2;
omegamin = -1;  omegamax = 1;

% The control is six-dimensional in this formulation.  
% The "real" controls are u1-u2 and u3-u4.  In order to
% use this formulation it is necessary to include the 
% control constraints ui>=0 (i=1,...4) along with the 
% path constraints u1+u2<=1 and u3+u4<=1.
u_min = -5000; u_max = 5000;

bounds.phase.initialtime.lower = t0;
bounds.phase.initialtime.upper = t0;
bounds.phase.finaltime.lower   = tf-t_err;
bounds.phase.finaltime.upper   = tf+t_err;

bounds.phase.initialstate.lower = [x0,y0,theta0,vx0,vy0,omega0];
bounds.phase.initialstate.upper = [x0,y0,theta0,vx0,vy0,omega0];
bounds.phase.state.lower        = [xmin,ymin,thetamin,vxmin,vymin,omegamin];
bounds.phase.state.upper        = [xmax,ymax,thetamax,vxmax,vymax,omegamax];
bounds.phase.finalstate.lower   = [xf,yf,thetaf,vxf,vyf,omegaf]-x_err';
bounds.phase.finalstate.upper   = [xf,yf,thetaf,vxf,vyf,omegaf]+x_err';
bounds.phase.control.lower = ones(1,obj.getNumInputs)*u_min;
bounds.phase.control.upper = ones(1,obj.getNumInputs)*u_max;
%TODO look up what these represent
bounds.phase.integral.lower = 0;
bounds.phase.integral.upper = 1000000000;
bounds.phase.path.lower = [(auxdata.r+(1/2)^0.5*0.1)^2];
bounds.phase.path.upper = [10000];

%tguess = solution. state
%u guess = solution. u etc.
tGuess     = [t0; tf];
xGuess     = [x0; xf];
yGuess     = [y0; yf];
thetaGuess = [theta0; thetaf];
vxGuess    = [vx0; vxf];
vyGuess    = [vy0; vyf];
omegaGuess = [omega0; omegaf];
u_guess = zeros(2,obj.getNumInputs);

guess.phase.time = tGuess;
guess.phase.state = [xGuess,yGuess,thetaGuess,vxGuess,vyGuess,omegaGuess];
guess.phase.control = u_guess;
guess.phase.integral = 0;

mesh.method = 'hp-PattersonRao'; %'hp-LiuRao';
mesh.tolerance = 1e-2; 
mesh.maxiteration = 1;
mesh.colpointsmin = 3;
mesh.colpointsmax = 10;
mesh.phase.colpoints = 4*ones(1,10);
mesh.phase.fraction = 0.1*ones(1,10);

% ----------------------------------------------------------------------- %
% Set up solver
% ----------------------------------------------------------------------- %
setup.name = 'Induction_Inspector';
%setup.functions.continuous = @planarDynamics;
setup.functions.continuous = @drakeifiedPlanarDynamics;
setup.functions.endpoint = @planarDynamicsEndpoint;

setup.auxdata = auxdata;

setup.bounds = bounds;

setup.guess = guess;

setup.mesh = mesh;


setup.nlp.solver = 'snopt'; % {'ipopt','snopt'}
setup.nlp.ipoptoptions.maxiterations = 100; 
setup.nlp.snoptoptions.maxiterations = 200;
setup.nlp.ipoptoptions.tolerance = 1e-3;
setup.nlp.snoptoptions.tolerance = 1e-5;

setup.derivatives.supplier = 'sparseCD';%'sparseFD'; %{'sparseCD';}
setup.derivatives.derivativelevel = 'second';

setup.displaylevel = 2;
setup.scales.method = 'automatic-bounds';

setup.method = 'RPM-Integration';

% ----------------------------------------------------------------------- %
% Solve problem and extract solution
% ----------------------------------------------------------------------- %
output = gpops2(setup);