%% Test gridding up the workspace
load('coupler_splines.mat');
splines.x = f_x_spline;
splines.y = f_y_spline;
mean_noise = 0.0001;
g_max = findMaxEffectiveDistance(mean_noise,splines);
edge_size = 0.1;
radius = 7;
bounding_box = [0.5 0.5 0.5];
[grid] = defineWorkspaceGrid( radius, g_max, bounding_box, edge_size);
figure(123);
plotGrid(grid, edge_size);
axis square