%% Inspector Explorer 


%% Specify the inspector 
RADIUS = 5;


d1 = 0.5*[1/2^.5,1/2^.5,-0.1];
d2 = 0.5*[-1/2^.5,1/2^.5,-0.1];
d3 = 0.5*[1/2^.5,-1/2^.5,-0.1];

%Generate a target force
xTarget = rand(1);
yTarget = rand(1);
zTarget = -0.05 + 0.1*rand(1);
f_target = [xTarget;yTarget;zTarget];  %Target Force


a1 = d1/norm(d1);
a2 = d2/norm(d2);
a3 = d3/norm(d3);

a = [a1;a2;a3];
d = [d1;d2;d3];

gen = InspectorGenerator();
gen = gen.setCouplers(a,d);
gen.genFile('/home/ben/drake-space/@InductionInspector/models/testest.urdf');
r = InductionInspector('./models/testest.urdf');
%Add a sphere
r = r.addSphere(RADIUS);
v = r.constructVisualizer();


%% Visualize
%% Display possible forces
% X = [0;0;0.1;zeros(15,1)];
% x0 = X;
% state_dims = [1:6];
% load('~/drake-space/@RigidBodyCoupler/coupler_splines.mat');
% callback = @(x,model)forceCallback(x,model,f_x_spline,f_y_spline);
% v = v.setCallback(callback);
% v.inspector(x0,state_dims)
