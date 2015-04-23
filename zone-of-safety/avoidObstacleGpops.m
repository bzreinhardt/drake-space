function output = avoidObstacleGpops(obj,x0)


%Finds a series of control inputs to avoid crashing

t0 = 0;
tf = 10;
t_err = 10;
x_err = [0.001;0.001;0.001;0;0;0];
u_min = -10000; 
u_max = 10000;

auxdata.r = obj.sphere_radius; %sphere radius
auxdata.c = obj.sphere_center;
auxdata.gamma = 1;
auxdata.a = obj.a;
auxdata.d = obj.d;

bounds.phase.initialtime.lower = t0;
bounds.phase.initialtime.upper = t0;
bounds.phase.initialstate.lower = [x0(1),x0(2),x0(3),x0(4),x0(5),x0(6)];
bounds.phase.initialstate.upper = [x0(1),x0(2),x0(3),x0(4),x0(5),x0(6)];
bounds.phase.finaltime.lower   = tf-t_err;
bounds.phase.finaltime.upper   = tf+t_err;
%try replacing inf with big nubmer
bounds.phase.state.upper = [1000000*ones(1,6)];
bounds.phase.state.lower = [-10000000*ones(1,6)];
bounds.phase.finalstate.upper = [1000000*ones(1,3),(zeros(1,3))];
bounds.phase.finalstate.lower = [-1000000*ones(1,3),(zeros(1,3))];

bounds.phase.control.lower = ones(1,obj.getNumInputs)*u_min;
bounds.phase.control.upper = ones(1,obj.getNumInputs)*u_max;

bounds.phase.path.lower = [(auxdata.r+0.01)^2];
bounds.phase.path.upper = [10000];

bounds.phase.integral.lower = 0;
bounds.phase.integral.upper = 1000000000;

tGuess     = [t0; tf];
u_guess = zeros(2,obj.getNumInputs);

guess.phase.time = tGuess;
guess.phase.state = [reshape(x0,1,6);reshape(x0,1,6)];
guess.phase.control = u_guess;
guess.phase.integral = 0;
%TODO add state to guess
mesh.method = 'hp-PattersonRao'; %'hp-LiuRao';
mesh.tolerance = 1e-5; 
mesh.maxiteration = 3;
mesh.colpointsmin = 3;
mesh.colpointsmax = 15;
mesh.phase.colpoints = 4*ones(1,10);
mesh.phase.fraction = 0.1*ones(1,10);

setup.name = 'Induction_Inspector';
%setup.functions.continuous = @planarDynamics;
setup.functions.continuous = @drakeifiedPlanarDynamics;
setup.functions.endpoint = @planarDynamicsEndpoint;
%
setup.auxdata = auxdata;

setup.bounds = bounds;

setup.guess = guess;

setup.mesh = mesh;

setup.nlp.solver = 'snopt'; % {'ipopt','snopt'}
setup.nlp.ipoptoptions.maxiterations = 100; 
setup.nlp.snoptoptions.maxiterations = 200;
setup.nlp.ipoptoptions.tolerance = 1e-2;
setup.nlp.snoptoptions.tolerance = 1e-4;

setup.derivatives.supplier = 'sparseCD';%'sparseFD'; %{'sparseCD';}
setup.derivatives.derivativelevel = 'second';

setup.displaylevel = 2;
setup.scales.method = 'automatic-bounds';

setup.method = 'RPM-Integration';

% ----------------------------------------------------------------------- %
% Solve problem and extract solution
% ----------------------------------------------------------------------- %
output = gpops2(setup);
end