function fitness = inspectorFitFun(params,p)
%params - arm angles
% consts
a = [zeros(2,length(params)); ones(2,length(params))]; %axes all in +z
l = 0.1; %arm legnth
d = zeros(3,length(params));
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
for i = 1:length(params)
    d(:,i) = [l*cos(params(i));l*sin(params(i));0];
    % p - struct with options
    % p.pend_opts - struct with options for pendulum fit functions
    
end
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
