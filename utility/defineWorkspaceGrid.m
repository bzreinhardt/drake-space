function [grid] = defineWorkspaceGrid( radius, g_max, params, edge_size, varargin)
%DEFINEWORKSPACEGRID defines a grid over the workspace of an actuation
%inspector. The edges of the grid are where the forces that can be produced
%by the actuator are on the same order as the mean disturbance from the
%environment.

%Bounding box - w,l,h of the inspector

%@retval - grid - center of the grid points


%assume the top of the curve is at the origin
divs = 20;
d = params.d;
%maximum distances that get from the center of mass
xbox = max(abs(d(:,1)));
ybox = max(abs(d(:,1)));
zbox = max(abs(d(:,1)));

z_max = g_max + zbox;
% assuming the z center is located at -r
x_max = sqrt(radius^2 - (-g_max + radius)^2);

z = 0:edge_size:z_max;
x = -x_max:edge_size:x_max;

[X,Z] = ndgrid(x,z);
grid = {X,Z};
end

