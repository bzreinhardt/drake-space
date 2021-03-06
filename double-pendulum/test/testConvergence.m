function testConvergence
    t1_range = [0 2*pi];
    t2_range = [0 2*pi];

    dt1_range = [0 0];
    dt2_range = [0 0];

    range = [t1_range; t2_range; dt1_range; dt2_range];
    function pend = createFun()
        [m,l] = generatePendParams;
        pend = DoublePendPlant(l,m);
    end

    function [fitness] = fitFun(pend)
        prm = LQRPRM(pend,range);
        prm.regions_max = 20;
        options = struct;
        options.method = 'tilqr';
        prm = prm.fillRegion(options);
        fitness = prm.volume;
        disp('volume = ');
        disp(fitness);
    end

create_fun = @()createFun();
    fit_fun = @(design)fitFun(design);

max_iterations = 40;
i = 1;
converged = 0;
folder = '/home/ben/drake-space/data/pendulum_convergence_test/';
[baseline.controller,baseline.design,baseline.its] = ...
            readControllerDesign(strcat(folder,'human-baseline.json'));
%load the baseline

while ~converged && i < max_iterations
    it = 25*i;
    fprintf(' Running test with %d iterations',it);
    ev_options.max_no_improve = it;
    ev_options.max_iterations = it;
    [best_design, max_fitness] = runEvolution(fit_fun,create_fun,ev_options);
    opt_prm = LQRPRM(best_design,range);
    opt_prm.regions_max = 20;
    options = struct;
    options.method = 'tilqr';
    
    opt_prm = opt_prm.fillRegion(options);
    opt_prm.findVolume;
    if (1-opt_prm.getVolume()/baseline.controller.volume) > 0.95
        converged = 1;
    end
    
    date_string = datestr(now);
    date_string((ismember(date_string,' ') == 1)) = '-';
    date_string((ismember(date_string,':') == 1)) = '_';
    filename = strcat(folder, ...
        date_string,'.json');
    notes.iterations = it;
    
    opt_prm.save(filename,notes);
    i = i+1;
end


end