%% test findActuator Potential

d1 = 0.5*[1/2^.5,1/2^.5,-0.1];
d2 = 0.5*[-1/2^.5,1/2^.5,-0.1];
d3 = 0.5*[1/2^.5,-1/2^.5,-0.1];
a1 = d1/norm(d1);
a2 = d2/norm(d2);
a3 = d3/norm(d3);

a = [a1;a2;a3];
d = [d1;d2;d3];
%define the sphere
r = 7;
c = [0;0;-7];
edge_size = 0.1;
params.a = a;
params.d = d;
load('coupler_splines.mat');
params.spline.x = f_x_spline;
params.spline.y = f_y_spline;
world.r = r;
world.c = c;
q = [0;0;0.1;zeros(9,1)];
lims = findActuatorPotential(q,params,world);

%iterate actuator potential over the grid region

g_max = findMaxEffectiveDistance(mean_noise,params.spline);
mean_noise = 0.0001;
bounding_box = [0.2 0.2 0.2];
[grid] = defineWorkspaceGrid( radius, g_max, params, edge_size);
X = grid{1};
Y = zeros(size(X));
Z = grid{2};
weight = zeros(size(X));

disp('finding actuation potentials');
for i = 1:numel(X)
    disp('idx: ');
    disp(i);
    q = [X(i);0;Z(i);zeros(9,1)];
    lims = findActuatorPotential(q,params,world);
    % heuristically determin actuation quality
    lims(abs(lims)<mean_noise) = 0;
    doa = numel(lims(lims~=0)); %degrees of actuation
    weight(i) = doa/6;
end
figure(12345); 
plotWeightedGrid(grid,edge_size,weight);

    


