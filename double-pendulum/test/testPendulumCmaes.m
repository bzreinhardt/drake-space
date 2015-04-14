function testPendulumCmaes
%Demonstrate convergence of lqr bubble algorithm
%%%%%%%%%%% set up cmaes parameters %%%%%%%
mass_range = [0.5 1.5];
length_range = [0.5 1.5];
opts.LBounds = [mass_range(1)*ones(2,1);length_range(1)*ones(2,1)];
opts.UBounds = [mass_range(2)*ones(2,1);length_range(2)*ones(2,1)];
%opts.StopIter = 500;
opts.LogPlot = 'on';
   
X0 = [mean(mass_range)*ones(2,1);mean(length_range)*ones(2,1)];
sigma = 0.5*ones(4,1);

%%%%%% Set up LQRPRM parameters %%%%%%%%%%%%%%%%%
%state parameters for equilibrium points to test%
    t1_range = [0 2*pi];
    t2_range = [0 2*pi];
    dt1_range = [0 0];
    dt2_range = [0 0];
    range = [t1_range; t2_range; dt1_range; dt2_range];
    %regions
    MAX_REGIONS = 20;
    p.pend_opts.max_regions = MAX_REGIONS;
    p.pend_opts.range = range;



%load the baseline
folder = '/home/ben/drake-space/data/pendulum_convergence_test/';
[baseline.controller,baseline.design,baseline.its] = ...
            readControllerDesign(strcat(folder,'human-baseline.json'));

%%%% Run CMAES %%%%%
[XMIN, FMIN, COUNTEVAL, STOPFLAG, OUT, BESTEVER] = cmaes('paramFitFun',X0,sigma,opts,p);
%Compare to best design
    opt_pend = DoublePendPlant([baseline.design.m1;baseline.design.m2],[baseline.design.l1;baseline.design.l2]);
    opt_prm = LQRPRM(opt_pend,range);
    opt_prm.regions_max = MAX_REGIONS;
    options = struct;
    options.method = 'tilqr';
    
    opt_prm = opt_prm.fillRegion(options);
    opt_prm = opt_prm.findVolume;
    
 % saving stuff   
%     date_string = datestr(now);
%     date_string((ismember(date_string,' ') == 1)) = '-';
%     date_string((ismember(date_string,':') == 1)) = '_';
%     filename = strcat(folder, ...
%         date_string,'.json');
%     notes.iterations = it;
% opt_prm.save(filename,notes);

disp(strcat('CMAES params = ',num2str(XMIN)));
disp(strcat('CMAES fitness = ',num2str(FMIN)));
disp(strcat('OPT fitness = ',num2str(opt_prm.volume)));

end