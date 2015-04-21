function fitness = inspectorFitFun(params,p)
%Generates fitness metric for 2d inspector with cmaes
%params - arm angles
% consts
%parse inputs
if nargin > 1
range = p.range;
max_regions = p.max_regions;
else
    x_range = [-0.05 0.05];
    y_range = [0.11 0.15];
    theta_range = [0 0];
    dx_range = [0 0];
    dy_range = [0 0];
    dtheta_range = [0 0];
    range = [x_range; y_range; theta_range; dx_range; dy_range; dtheta_range];
    max_regions = 20;
end

%set up parameters
[a,d] = armAnglesToInspectorParams(params);
%generate inspector
insp = Inspector2d(a,d);
prm = LQRPRM(insp,range);
prm.regions_max = max_regions;
options = struct;
options.method = 'tilqr';
try
    prm = prm.fillRegion(options);
    fitness = -prm.volume;
catch err
    disp(err)
    fitness = -10000; 
end 
end
