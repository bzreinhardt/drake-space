function [range,full] = findOptimalPendVol(max_regions,N)
    if nargin < 1
        max_regions = 20;
    end
    if nargin < 2
        N = 100;
    end
    
    t1_range = [0 2*pi];
    t2_range = [0 2*pi];
    dt1_range = [0 0];
    dt2_range = [0 0];
    range = [t1_range; t2_range; dt1_range; dt2_range];
    
    
    p.pend_opts.max_regions = max_regions;
    p.pend_opts.range = range;
    
 
    full = zeros(20,1);
    for i = 1:N
       	v = paramFitFun(ones(4,1)*0.5,p);
        full(i) = v;
    end
    range = [min(full) max(full)];
    savejson('',full,'optimal_pend_vols.json');
end