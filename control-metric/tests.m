%%%% Toy problems for generating control space %%%%
%generate a sphere, find it's volume with monte carlo and halton
%integration

%TODO - run testConnectControlRealDynamics 9:33am 1/1/2015
%TODO - stick the checkCOntrollersInRegion in the generate and connect
%regions 1/1 3:26

%test plan
% have prms prune themselves of unconnected volumes
%random fitness optimization working
% generate fitness based on controllable volume
% evolve fitness based on controllable volume for non-lqr spheres
% evolve fitness based on controllable volume for lqr spheres


classdef tests
methods (Static)
    function testMonteCarlo
        %% test monte carlo
        %check value of pi
        dims = 2;
        sys = Inspector2d();
        range = repmat([-1 1],dims,1);
        prm = LQRPRM(sys,range);
        x = msspoly('x',dims);
        ellipse = x(1)^2+x(2)^2;
        occ1 =@(state)((state(1)^2 + state(2)^2) <= 1);
        occ2 = @(state1,state2)(full(dmsubs(ellipse,x,[state1;state2]) < 1));
        func = fn(ellipse,x);
        occ3 = @(state)(func(state) < 1);
        occ4 = @(state1,state2)(func([state1;state2]) < 1);
        h = sfigure(1337);
        clear h;
        y = getProjection(x,ellipse,zeros(dims,1),[1,2]);
        h=fill3(y(1,:),y(2,:),repmat(0,1,size(y,2)),'r','LineStyle','-','LineWidth',2);
        
        options.arrayfun = true;
        
        % evaluating dsubs is sloooow
        
        disp('checking random');
        options.rand_method = 'rand';
        Q1 = prm.monteCarlo(occ1,range,options);
        disp(fprintf('Q1 = %d',Q1));
        valuecheck(Q1,nballVol(1,2),0.1*nballVol(1,2));
        
        disp('checking halton');
        options.rand_method = 'halton';
        Q2 = prm.monteCarlo(occ1,range,options);
        disp(fprintf('Q2 = %d',Q2));
        valuecheck(Q2,nballVol(1,2),0.1*nballVol(1,2));
        
        disp('checking higher dimensions');
                 dims = 6;
         range6 = repmat([-1 1],dims,1);
            x = msspoly('x',dims);
         hyperellipse = x(1)^2+x(2)^2+x(3)^2+x(4)^2+x(5)^2+x(6)^2;
         [~,p,M,sz]=decomp(hyperellipse);
         
         occ6 = str2func(strcat('@(x)(',decomp2fun(M,p,1,true),'<=1)'));
         Q6 = prm.monteCarlo(occ6,range6);

         disp(fprintf('Q6 = %d',Q6));
         valuecheck(Q6, nballVol(1,6),0.1*nballVol(1,6));
         
        
    end

    
    function testGetBoundingBox
        sys = Inspector2d();
        x0 = [-0.5 0.5;0 0];
        prm = LQRPRM(sys);
        options = struct();
        options.plot = 'static';
        figure(1337);clf;
        options.method = 'sphere2d';
        for i=1:size(x0,2)
            options.x0 = x0(:,i);
            prm = prm.genControlRegion(options);
        end
        bb = prm.getBoundingBox;
        disp('Bounding Box: ');
        disp(bb);
        valuecheck(bb, [-1.5 1.5; -1 1; zeros(4,2)], 1E-3);
    end


    function testSphereOverlap
        %2d first
        
        dims = 2;
        sys = Inspector2d();
        range = repmat([-1 1],dims,1);
        prm = LQRPRM(sys,range);
        x = msspoly('x',dims);
        r1 = 0.75;
        ellipse1 = 1/r1^2*(x(1)-(1-r1))^2+(4*x(2))^2;
        ellipse2 = 1/r1^2*(x(1)-(-1+r1))^2+(4*x(2))^2;
        
        %plot ellipse
        h = sfigure(1337);
        
        clf;
        y1 = getProjection(x,ellipse1,zeros(dims,1),[1,2]);
        h=fill3(y1(1,:),y1(2,:),repmat(0,1,size(y1,2)),'r','LineStyle','-','LineWidth',2);
        hold on;
        y2 = getProjection(x,ellipse2,zeros(dims,1),[1,2]);
         h=fill3(y2(1,:),y2(2,:),repmat(0,1,size(y2,2)),'g','LineStyle','-','LineWidth',2);
        view(2);
        hold off;
        
        func1 = fn(ellipse1,x);
        func2 = fn(ellipse2,x);
        
        occ = @(state)([func1(state) <= 1;func2(state) <= 1]);
        options.plot = 'static';
        tic
        [Q,overlaps] = prm.monteCarlo(occ,range,options);
        toc
        disp(overlaps);
        
        
        
        
        
    end
   %test control region bounding box 
    function testControlRegionBB
        sys = Inspector2d;
        x0 = [0;0];
        c = ControlRegion(x0,sys);
        state = sys.getStateFrame.getPoly; 
        c.x = state(1:2);
        c.V = state(1)^2+state(2)^2;
        valuecheck(c.getBoundingBox,[-1 1;-1 1],0.01);
        c.x0 = [0;1];
        valuecheck(c.getBoundingBox,[-1 1; 0 2],0.01);
x0 = [0;0.11;zeros(sys.getNumStates-2,1)];
        [control,V0] = findLQR(sys,x0,...
            zeros(sys.getNumInputs,1));
        c.V = V0;
        c.x0 = x0;
        disp(c.getBoundingBox);
        
    end

function testAddControlRegion
sys = Inspector2d;
x0 = [0 0 1; 0 1 0];
prm = LQRPRM(sys);
options = struct();
options.method = 'sphere2d';
for i = 1:size(x0,2)
    options.x0 = x0(:,i);
    prm = prm.genControlRegion(options);
    c = prm.regions{i};
    valuecheck(c.x0,x0(:,i));
    disp(class(c.V));
    hold on;
    c.draw; drawnow; hold off;
    
    
    range = c.getBoundingBox();
    hold on;
    plot([range(1,1) range(1,1) range(1,2) range(1,2) range(1,1)],...
        [range(2,1) range(2,2) range(2,2) range(2,1) range(2,1)],...
        'LineWidth',3);
    pause(1);
end
valuecheck(numel(prm.regions), size(x0,2));
end

function testConnectControlRegion1
sys = Inspector2d;
x0 = [0; 0];
prm = LQRPRM(sys);
options = struct();
options.method = 'sphere2d';
options.x0 = x0;

figure(1337);clf;
prm = prm.genControlRegion(options);

prm = prm.findVolume();
valuecheck(prm.getVolume, nballVol(1,2),0.1*nballVol(1,2));
valuecheck(size(prm.occupancy_map),[1 1]);

end



function testConnectControlRegion2
    %area of two intersecting circles: 
    %A = A(R,d1)+A(r,d2) http://mathworld.wolfram.com/Circle-CircleIntersection.html
    %0.5*sqrt((-d+r+R)*(d+r-R)*(d-r+R)*(d+r+R)) where d is distance between
    %centers
    sys = Inspector2d;
    x0 = [-0.5 0.5;0 0];
    prm = LQRPRM(sys);
    options = struct();
    options.plot = 'static';
    figure(1337);clf;
    options.method = 'sphere2d';
    for i=1:size(x0,2)
        options.x0 = x0(:,i);
        prm = prm.genControlRegion(options);
    end
    fprintf('total volue = %f \n',prm.volume);
    valuecheck(size(prm.occupancy_map),[2,2]);
    disp(prm.occupancy_map);
    
    
end

function testConnectControlRegion3
    sys = Inspector2d;
    x0 = [-1 0.5 1.2;0 0 0.1];
    prm = LQRPRM(sys);
    options = struct();
    options.plot = 'static';
    figure(1337);clf;
    options.method = 'sphere2d';
    for i=1:size(x0,2)
        options.x0 = x0(:,i);
        prm = prm.genControlRegion(options);
    end
    fprintf('\n total volume = %f \n',prm.volume);
    valuecheck(size(prm.occupancy_map),[3,3]);
    disp(prm.occupancy_map);
    prm.draw;
end

function testConnectControlRegion6d
    sys = Inspector2d;
    x0 = [[-2; zeros(5,1)],[2; zeros(5,1)]];
    prm = LQRPRM(sys);
    options = struct();
    options.plot = 'static';
    figure(1337);clf;
    options.method = 'sphere6d';
    for i=1:size(x0,2)
        options.x0 = x0(:,i);
        prm = prm.genControlRegion(options);
    end
    fprintf('\n total volume = %f \n',prm.volume);
    valuecheck(size(prm.occupancy_map),[2,2]);
    disp(prm.occupancy_map);
end

function testConnectControlRegion6d2
    sys = Inspector2d;
    x0 = [[-2; zeros(5,1)],[-1;zeros(5,1)],[2; zeros(5,1)]];
    prm = LQRPRM(sys);
    options = struct();
    options.plot = 'static';
    figure(1337);clf;
    options.method = 'sphere6d';
    for i=1:size(x0,2)
        options.x0 = x0(:,i);
        prm = prm.genControlRegion(options);
    end
    fprintf('\n total volume = %f \n',prm.volume);
    valuecheck(size(prm.occupancy_map),[3,3]);
    disp(prm.occupancy_map);
end

%TODO

function testConnectControlRealDynamics %works 1/1
    sys = Inspector2d;
    x_range = [-0.05 0.05];
    y_range = [0.10 0.12];
    theta_range = [0 0];
    dx_range = [0 0];
    dy_range = [0 0];
    dtheta_range = [0 0];
    range = [x_range; y_range; theta_range; dx_range; dy_range; dtheta_range];
    x0 = [[-0.01;0.11; zeros(4,1)],[0.01;0.11; zeros(4,1)]];
    prm = LQRPRM(sys,range);
    options = struct();
    %options.plot = 'static';
    options.method = 'tilqr';
    options.normalize = false;
    

    figure(1337);clf;
    for i=1:size(x0,2)
        options.x0 = x0(:,i);
        prm = prm.genControlRegion(options);
       

    end
    disp('fast method')
    tic
     prm = prm.findVolume(options);
     disp(prm.volume);
     toc
     disp('slow method')
     tic
     options.slow_occ_test = true;
     prm = prm.findVolume(options);
     disp(prm.volume);
     toc
     
    fprintf('\n total volume = %f \n',prm.volume*1E10);
    valuecheck(size(prm.occupancy_map),[2,2]);
    disp(prm.occupancy_map);
    figure(1337);clf;
    prm.draw;
 %   prm.save;
end

function testPlotGraph
    
    sys = Inspector2d;
    x_range = [-0.05 0.05];
    y_range = [0.10 0.12];
    theta_range = [0 0];
    dx_range = [0 0];
    dy_range = [0 0];
    dtheta_range = [0 0];
    range = [x_range; y_range; theta_range; dx_range; dy_range; dtheta_range];
    x0 = [[-0.01;0.11; zeros(4,1)],[0.01;0.11; zeros(4,1)]];
    prm = LQRPRM(sys,range);
    options = struct();
    options.method = 'tilqr';
    options.normalize = true;
    

    figure(1337);clf;
    for i=1:size(x0,2)
        options.x0 = x0(:,i);
        prm = prm.genControlRegion(options);
        prm = prm.findVolume(options);
    end
    prm.plotGraph;
    
end
%TODO - test a random point generator
function testGenRandomMap
    sys = Inspector2d;
    range = [-3 3; -3 3];
    prm = LQRPRM(sys,range);
    options = struct;
    options.method = 'sphere2d';
    figure(1337); clf;
    prm = prm.fillRegion(options);
    disp(prm.occupancy_map);
    disp(prm.volume);

    prm.draw(1337);
    
    prm2 = LQRPRM(sys,range);
    options = struct;
    options.method = 'sphere2d';
    options.normalize = 'true';
    figure(1338); clf;
    prm2 = prm2.fillRegion(options);
    disp(prm2.occupancy_map);
    disp(prm2.volume);
    prm2.draw(1338);

end

%Test random generation with real dynamics
function testGenRandomMapRealDynamics
    sys = Inspector2d;
    %note: range has to include only stable points (ie. here the velocities
    %have to be zero) 
    x_range = [-0.05 0.05];
    y_range = [0.10 0.12];
    theta_range = [0 0];
    dx_range = [0 0];
    dy_range = [0 0];
    dtheta_range = [0 0];
    
    range = [x_range; y_range; theta_range; dx_range; dy_range; dtheta_range];
    
    prm = LQRPRM(sys,range);
    options = struct;
    options.method = 'tilqr';
    figure(1337); clf;
    prm = prm.fillRegion(options);
    disp(prm.occupancy_map);

    disp('Volume * 1E10');
    disp(prm.volume*1E10);

    prm.draw;
end

function testSave
    cd ~/drake-space/data
    sys = Inspector2d;
    %note: range has to include only stable points (ie. here the velocities
    %have to be zero) 
    x_range = [-0.05 0.05];
    y_range = [0.10 0.12];
    theta_range = [0 0];
    dx_range = [0 0];
    dy_range = [0 0];
    dtheta_range = [0 0];
    
    range = [x_range; y_range; theta_range; dx_range; dy_range; dtheta_range];
    
    prm = LQRPRM(sys,range);
    options = struct;
    options.method = 'tilqr';
    figure(1337); clf;
    prm = prm.fillRegion(options);
    disp(prm.occupancy_map);

    disp('Volume * 1E10');
    disp(prm.volume*1E10);

    prm.draw;
    prm.save;
end



%TODO - test that the area will converge and be pruned
function testMapConvergence
end

%TODO - test finding the convex hull of the reachable space
function testConvexHull
end

%Stopping point - need to do iterative thing
%need to think about convergence, how to pick random points
function testRandomGen
    %TODO, need to debug why random generation isn't working 1/3
    for i = 1:5
        clear v; clear insp;
        [a,d]= generateCouplerParams();
        disp('a and d');
        disp(a);
        disp(d);
        insp = Inspector2d(a,d);
        v = Inspector2dVisualizer(insp);
        figure(25);clf;
        v.draw(0,[0;0.11;zeros(4,1)]);
         drawnow;
         pause(3);
    end
end

%check that random generation will converge on the correct solution given
%an arbitrary fitness function
function testRandomFitness
    
    function insp = createFun()
        [a,d] = generateCouplerParams();
        insp = Inspector2d(a,d);
    end
    create_fun = @()createFun();
    fit_fun = @(design)fakeFitness(design);
    [best_design, max_fitness] = runEvolution(fit_fun,create_fun);
    v = Inspector2dVisualizer(best_design);
    best_fitness = 1;
    disp('percent of max');
    disp(max_fitness/best_fitness*100);
    figure(25);clf;
    v.draw(0,[0;0.11;zeros(4,1)]);
         drawnow;
    
end

function testGA

    A = [eye(4);-eye(4)];
    b = [zeros(4,1);pi*ones(4,1)];
    fun = @(x)(gaFitFun(x));
    [x, fval] = ga(fun,4,A,b);
    max_fitness = -fval;
    a = [zeros(2,4);ones(1,4)];
        d = zeros(3,4);
    for i = 1:4
        d(:,i) = 0.1*[cos(x(i));sin(x(i));0];
    end
    best_design = Inspector2d(a,d);
    v = Inspector2dVisualizer(best_design);
    best_fitness = 1;
    disp('percent of max');
    disp(max_fitness/best_fitness*100);
    figure(25);clf;
    v.draw(0,[0;0.11;zeros(4,1)]);
         drawnow;
end


function testGARandomControllerRealDynamics
     x_range = [-0.05 0.05];
    y_range = [0.10 0.15];
    theta_range = [0 0];
    dx_range = [0 0];
    dy_range = [0 0];
    dtheta_range = [0 0];
    range = [x_range; y_range; theta_range; dx_range; dy_range; dtheta_range];
    
    function [fitness] = fitFun(params)
        
        a = [zeros(2,4);ones(1,4)];
        d = zeros(3,4);
        for i = 1:4
        d(:,i) = 0.1*[cos(params(i));sin(params(i));0];
        end
        
        insp = Inspector2d(a,d);
        prm = LQRPRM(insp,range);
        prm.regions_max = 20;
        options = struct;
        options.method = 'tilqr';
        %options.normalize = 'true';
        prm = prm.fillRegion(options);
        fitness = -prm.volume;
%         disp('volume = ');
%         disp(fitness);
    end
    
    tic
     A = [eye(4);-eye(4)];
    b = [zeros(4,1);pi*ones(4,1)];
    fun = @(x)(fitFun(x));
    ga_options = gaoptimset('StallGenLimit',25,'StallTimeLimit',100);
    [x, fval] = ga(fun,4,A,b,[],[],[],[],[],ga_options);
    toc
    a = [zeros(2,4);ones(1,4)];
        d = zeros(3,4);
    for i = 1:4
        d(:,i) = 0.1*[cos(x(i));sin(x(i));0];
    end
    best_design = Inspector2d(a,d);
    max_fitness = -fval;
    
    opt_prm = LQRPRM(best_design,range);
    opt_prm.regions_max = 20;
    options = struct;
    options.method = 'tilqr';
    options.rand_method = 'rand';
    opt_prm = opt_prm.fillRegion(options);
    max_fitness2 = opt_prm.volume;

   
    
    
     % Find baseline things
    baseline = Inspector2d; baseline = baseline.setBaseline('four_coupler');
    baseline_prm = LQRPRM(baseline,range);
    baseline_prm.regions_max = 20;
    options = struct;
    options.method = 'tilqr';
    baseline_prm = baseline_prm.fillRegion(options);
    baseline_fitness = baseline_prm.volume;
    
    
    %display values
    disp('controllable volume');
    disp(max_fitness);
    disp('recaluclated controllable volume');
    disp(max_fitness2);
    disp('baseline volume');
    disp(baseline_fitness);
    %visualize things
    figure(25);clf;
    v = Inspector2dVisualizer(best_design,25);
    v.draw(0,[0;0.11;zeros(4,1)]);
    title('');
    
    figure(26);clf;
    v2 = Inspector2dVisualizer(baseline,26);
    v2.draw(0,[0;0.11;zeros(4,1)]);
    title('');
    
    figure(27);clf;
    opt_prm.draw(27);
    hold on;
    title(sprintf('V_a = %d',max_fitness));
    
    figure(28);clf;
    baseline_prm.draw(28);
    title(sprintf('V_b = %d',baseline_fitness));
   
end
    

%randomly generate inspector parameters, generate a small prm for them, and
%and use volume as performance metric
function testRandomGenRandomFitnessRealDynamics
    
    x_range = [-0.05 0.05];
    y_range = [0.10 0.15];
    theta_range = [0 0];
    dx_range = [0 0];
    dy_range = [0 0];
    dtheta_range = [0 0];
    range = [x_range; y_range; theta_range; dx_range; dy_range; dtheta_range];

    function insp = createFun()
        [a,d] = generateCouplerParams(num_couplers);
        insp = Inspector2d(a,d);
    end

    
    function [fitness] = fitFun(insp)
        
        prm = LQRPRM(insp,range);
        prm.regions_max = 20;
        options = struct;
        options.method = 'tilqr';
        %options.normalize = 'true';
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
    options.rand_method = 'rand';
    opt_prm = opt_prm.fillRegion(options);
    max_fitness2 = opt_prm.volume;
    
    
    
    
    % Find baseline things
    baseline = Inspector2d; baseline = baseline.setBaseline('four_coupler');
    baseline_prm = LQRPRM(baseline,range);
    baseline_prm.regions_max = 20;
    options = struct;
    options.method = 'tilqr';
    baseline_prm = baseline_prm.fillRegion(options);
    baseline_fitness = baseline_prm.volume;
    
    
    %display values
    disp('controllable volume');
    disp(max_fitness);
    disp('recaluclated controllable volume');
    disp(max_fitness2);
    disp('baseline volume');
    disp(baseline_fitness);
    %visualize things
    figure(25);clf;
    v = Inspector2dVisualizer(best_design,25);
    v.draw(0,[0;0.11;zeros(4,1)]);
    title('');
    
    figure(26);clf;
    v2 = Inspector2dVisualizer(baseline,26);
    v2.draw(0,[0;0.11;zeros(4,1)]);
    title('');
    
    figure(27);clf;
    opt_prm.draw(27);
    hold on;
    title(sprintf('V_a = %d',max_fitness));
    
    figure(28);clf;
    baseline_prm.draw(28);
    title(sprintf('V_b = %d',baseline_fitness));
   

end

function findDesign(num_couplers)
    if nargin < 1
        num_couplers = 4;
    end
         x_range = [-0.05 0.05];
    y_range = [0.10 0.15];
    theta_range = [0 0];
    dx_range = [0 0];
    dy_range = [0 0];
    dtheta_range = [0 0];
    range = [x_range; y_range; theta_range; dx_range; dy_range; dtheta_range];

    function insp = createFun()
        [a,d] = generateCouplerParams(num_couplers);
        insp = Inspector2d(a,d);
    end

    
    function [fitness] = fitFun(insp)
        
        prm = LQRPRM(insp,range);
        prm.regions_max = 20;
        options = struct;
        options.method = 'tilqr';
        %options.normalize = 'true';
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
    opt_prm = opt_prm.fillRegion(options);
    opt_prm = opt_prm.findVolume;
    
    opt_prm.save;
   
end

function overnightTests
    min_actuators = 3;
    max_actuators = 8;
    tests = 10;
    for i = min_actuators:max_actuators
        for j = 1:tests
            
        end
        
    end
    
end

end

end


