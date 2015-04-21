function fitness = pendulumFitness(params,p)
%%%
%%%% Default Parameters
if nargin < 2
    %state parameters for equilibrium points to test%
    t1_range = [0 2*pi];
    t2_range = [0 2*pi];
    dt1_range = [0 0];
    dt2_range = [0 0];
    range = [t1_range; t2_range; dt1_range; dt2_range];
    %regions
    regions = 20;
else
    range = p.range;
    regions = p.regions;
end

pend = DoublePendPlant(params(1:2),params(3:4));
prm = LQRPRM(pend,range);
prm.regions_max = regions;
options = struct;
options.method = 'tilqr';
prm = prm.fillRegion(options);
fitness = -prm.volume;
%disp('volume = ');
%disp(fitness);
end