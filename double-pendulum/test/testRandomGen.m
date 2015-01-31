function testRandomGen
%TESTRANDOMGEN Summary of this function goes here
%   Detailed explanation goes here

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
    [best_design, max_fitness] = runEvolution(fit_fun,create_fun);
    
    opt_prm = LQRPRM(best_design,range);
    opt_prm.regions_max = 20;
    options = struct;
    options.method = 'tilqr';
    
    opt_prm = opt_prm.fillRegion(options);
    max_fitness2 = opt_prm.volume;
    
    baseline_design = DoublePendPlant([0.5;0.5],[0.5;0.5]);
    baseline_prm = LQRPRM(baseline_design,range);
    baseline_prm = baseline_prm.fillRegion(options);
    baseline_fitness = baseline_prm.volume;

    %display values
    disp('controllable volume');
    disp(max_fitness);
    disp('recaluclated controllable volume');
    disp(max_fitness2);
    disp('m = ');
    disp([best_design.m1, best_design.m2]);
    disp('l = ');
    disp([best_design.l1, best_design.l2]);
    
    disp('baseline volume');
    disp(baseline_fitness);
    disp('m = ');
    disp([baseline_design.m1, baseline_design.m2]);
    disp('l = ');
    disp([baseline_design.l1, baseline_design.l2]);
    
     %visualize things
    figure(25);clf;
    v = DoublePendVisualizer(best_design);
    v.draw(0,[zeros(4,1)]);
    title('');
    
     figure(26);clf;
    v = DoublePendVisualizer(baseline_design);
    v.draw(0,[zeros(4,1)]);
    title('');
    
    figure(27);clf;
    opt_prm.draw(27);
    hold on;
    title(sprintf('V_a = %d',max_fitness));
    
    figure(28);clf;
    baseline_prm.draw(28);
    hold on;
    title(sprintf('V_a = %d',baseline_fitness));

end

