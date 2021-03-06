<<<<<<< HEAD
classdef LQRPRM < HybridDrakeSystem
=======
classdef LQRPRM < HybridDrakeSystem 
>>>>>>> 5f12e3960ed647f1e5ace7049fc85481b8c48de0
    %LQRPRM holds the controllers the define a global control policy
    properties
        occupancy_map;
        volume; %normalized stabilizable volume in state space
        scales; %scales of each dimension in state space
        regions; %array of regional controllers
<<<<<<< HEAD
        tot_range; %range of each dimension in state space for generating controllers
=======
        tot_range; %range of each dimension in state space
>>>>>>> 5f12e3960ed647f1e5ace7049fc85481b8c48de0
        sys;
        regions_max; %maximum number of regions in the system
        
    end
    %Todo - build in state constraint checking
    methods
        %set up the system
        %@param sys - drakeSystem being controlled
        function obj = LQRPRM(sys,range)
            obj = obj@HybridDrakeSystem(sys.getNumInputs(),sys.getNumOutputs());
            obj = obj.setInputFrame(sys.getInputFrame());
            obj = obj.setOutputFrame(sys.getOutputFrame());
            
            if nargin>1 && ~isempty(range)
                obj.tot_range = range;
            elseif sys.getNumStates == 6
                %if there are 6 states, assume it's x,y,theta, and
                %derivatives
<<<<<<< HEAD
                obj.tot_range = [-0.1 0.1; 0.09 0.2; ...
=======
                obj.tot_range = [-0.1 0.1; 0.1 0.2; ...
>>>>>>> 5f12e3960ed647f1e5ace7049fc85481b8c48de0
                    -pi/2 pi/2; -0.1 0.1; ...
                    -0.1 0.1; -0.01 0.01];
            elseif sys.getNumStates == 12
            end
<<<<<<< HEAD
            %�?�?�????���??��?�????�?�?�
=======
            %�?�?�????���??��?�????�?�?�
>>>>>>> 5f12e3960ed647f1e5ace7049fc85481b8c48de0
            obj.sys = sys;
            obj.regions = {};
            obj.occupancy_map = sparse(0);
            obj.volume = 0;
            obj.regions_max = 10;
            
        end
        
        %populate the LQRPRM with controllers
        function obj = fillRegion(obj,options)
<<<<<<< HEAD
            
            %parse inputs
            if nargin <2
                options = struct();
                options.connect_at_end = 'true';
            end
            %this should probably be a property of the controller
            if ~isfield(options,'connect_at_end')
                options.connect_at_end = 'true';
            end
            
            %keep adding controllers
            while ~(numel(obj.regions)>= obj.regions_max)
                options.x0 = obj.generateCenter;
                obj = obj.genControlRegion(options);
                %          fprintf('Number of controllers: %d\n',numel(obj.regions));
            end
            
            % find the volume and connect all the controllers
            if options.connect_at_end
                
              [Q,overlaps] = findVolume(obj, options);
                obj.volume = Q;
                obj.occupancy_map = overlaps;
            end
            
            %prune controllers
        end
        
        function [Q,overlaps] = findVolume(obj, options)
            if nargin < 2
                options = struct();
            end
            %occupancy test to check against each ellipse
                occ_test = cell(numel(obj.regions),1);
                for i = 1:numel(obj.regions)
                    func = obj.regions{i}.getFn; %get the lyapunov polynomial
                    occ_test{i} = @(state)(func(state-obj.regions{i}.x0) <= 1); %see whether state is within level set
                end
                checkOcc = @(state)(cellfun(@(func)(func(state)),occ_test));
                bb = obj.getBoundingBox;
                [Q,overlaps] = obj.monteCarlo(checkOcc,bb,options);
                
=======
            %keep adding controllers
            if nargin <2
            options = struct();
            end
            while ~(numel(obj.regions)>= obj.regions_max)
                options.x0 = obj.generateCenter;
                obj = obj.genControlRegion(options);
      %          fprintf('Number of controllers: %d\n',numel(obj.regions));
            end
         
            %prune controllers 
>>>>>>> 5f12e3960ed647f1e5ace7049fc85481b8c48de0
        end
        
        function obj = genControlRegion(obj,options)
            %generate a random ellipse
            if nargin < 1
                options = struct();
                options.method = 'sphere2d';
                options.random = 'true';
<<<<<<< HEAD
                options.connect_at_end = 'true';
=======
>>>>>>> 5f12e3960ed647f1e5ace7049fc85481b8c48de0
            end
            %generate the cener of the control region
            if isfield(options,'x0')
                x0 = options.x0;
            else
                %pick a random box center (use heuristic here')
                x0 = obj.generateCenter();
            end
<<<<<<< HEAD
            if ~isfield(options,'connect_at_end')
                options.connect_at_end = 'true';
            end
=======
>>>>>>> 5f12e3960ed647f1e5ace7049fc85481b8c48de0
            c = ControlRegion(x0,obj.sys);
            c = c.generateController(options);
            obj.regions{1+numel(obj.regions)} = c;
            %debugging drawing
<<<<<<< HEAD
            if ~options.connect_at_end
                obj = obj.connectControlRegion(c,options);
            end
=======
            
            obj = obj.connectControlRegion(c,options);
            
            
            
>>>>>>> 5f12e3960ed647f1e5ace7049fc85481b8c48de0
        end
        %generate a new point for a control center
        function x0 = generateCenter(obj)
            x0 = (obj.tot_range(:,2)-obj.tot_range(:,1)).*rand(size(obj.tot_range,1),1)...
<<<<<<< HEAD
                +obj.tot_range(:,1);
        end
        %% Connect a gain node to the controller find edges and additional volume
        
        function obj = connectControlRegion(obj, c, options)
            %create a bounding box around c
            %do a monte carlo in that bounding box
            %connect c to
            if nargin < 3
                options = struct();
            end
            %construct the occupancy function
            %subtracting the equilibrium point from the input is the lazy way to do
            %it. The correct way is to do a frame transformation on the polynomial
            %beforehand
            bb = c.getBoundingBox();
            idx = obj.checkControllersInRegion(bb); %get the indexes of the other controllers that might overlap
            
            occ_test = cell(numel(obj.regions),1);
            for i = 1:numel(obj.regions)
                if all(idx-i)
                    occ_test{i} = @(state)(false);
                else
                    func = obj.regions{i}.getFn;
                    occ_test{i} = @(state)(func(state-obj.regions{i}.x0) <= 1);
                end
            end
            checkOcc = @(state)(cellfun(@(func)(func(state)),occ_test));
            
            options.focus = numel(obj.regions); %get the volume just from the most recently added ellipse
            [Q,overlaps] = obj.monteCarlo(checkOcc,bb,options);
            obj.volume = obj.volume+Q;
            combo = {obj.occupancy_map, overlaps};
            obj.occupancy_map = combineSparse(combo);
            %normalize the occ map
            obj.occupancy_map(obj.occupancy_map > 1) = 1;
        end
        %%
=======
                    +obj.tot_range(:,1);
        end
        
 function obj = connectControlRegion(obj, c, options)
    %create a bounding box around c
    %do a monte carlo in that bounding box
    %connect c to 
    if nargin < 3
        options = struct();
    end
    %construct the occupancy function
    %subtracting the equilibrium point from the input is the lazy way to do
    %it. The correct way is to do a frame transformation on the polynomial
    %beforehand
    bb = c.getBoundingBox();
    idx = obj.checkControllersInRegion(bb); %get the indexes of the other controllers that might overlap
     
    occ_test = cell(numel(obj.regions),1);
    for i = 1:numel(obj.regions)
         if all(idx-i)
            occ_test{i} = @(state)(false);
         else
            func = obj.regions{i}.getFn;
            occ_test{i} = @(state)(func(state-obj.regions{i}.x0) <= 1);
        end
    end
    checkOcc = @(state)(cellfun(@(func)(func(state)),occ_test));
    
    options.focus = numel(obj.regions); %get the volume just from the most recently added ellipse
     [Q,overlaps] = obj.monteCarlo(checkOcc,bb,options);
     obj.volume = obj.volume+Q;
     combo = {obj.occupancy_map, overlaps};
     obj.occupancy_map = combineSparse(combo);
     %normalize the occ map
     obj.occupancy_map(obj.occupancy_map > 1) = 1;
     
     
    %
    
end
>>>>>>> 5f12e3960ed647f1e5ace7049fc85481b8c48de0
        
        function stochasticIntegration(obj)
            %generate a point at random in the range
            %check it against each sphere
            
<<<<<<< HEAD
            
        end
        %% Draw all the control regions
        
        function draw(obj,fig)
            if nargin < 2
                fig = gcf;
            end
            figure(fig); hold on;
=======
            for i = 1:numel(regions)
                
            end
        end
        
        function draw(obj,fig)
            if nargin < 2
                fig = 1337;
            end
            figure(fig); hold on; 
>>>>>>> 5f12e3960ed647f1e5ace7049fc85481b8c48de0
            for i = 1:numel(obj.regions)
                obj.regions{i}.draw;
            end
            %two ways of doing it, the stochastic way and the not
            %stochastic way
            
        end
        %get the bounding box for the entire controller
<<<<<<< HEAD
        
        function bb = getBoundingBox(obj)

            bb = zeros(obj.sys.getNumStates,2);
            
            for i = 1:numel(obj.regions)
                c_bb = obj.regions{i}.getBoundingBox();
                % if c_bb is lower dimension, pad it to the higher
                % dimensional controller space
                if size(c_bb,1) < size(bb,1)
                    c_bb = [c_bb; zeros(size(bb,1)-size(c_bb,1),2)];
                end
                diff_bb = [bb(:,1) - c_bb(:,1), c_bb(:,2) - bb(:,2)];
                inds = find(diff_bb > 0);
                bb(inds) = c_bb(inds);
            end
=======
       
        function bb = getBoundingBox(obj)
            error('getBundingbox needs to be implemented.'); 
>>>>>>> 5f12e3960ed647f1e5ace7049fc85481b8c48de0
        end
        %check the possible controllers in a region
        function [idx] = checkControllersInRegion(obj,bb)
            idx = zeros(numel(obj.regions),1);
            for i = 1:numel(obj.regions)
                if checkTwoBB(obj.regions{i}.bb,bb);
                    idx(i) = 1;
                end
                idx = find(idx);
            end
        end
        
<<<<<<< HEAD
        
    end
    
    methods (Static)
        
        %montecarlo does a naive montecarlo integration
        function [Q,overlaps] = monteCarlo(checkOcc,range,options)
            %@param checkOcc - function handle that takes a point and
            %returns whether it's occupied or not
            %@param range - range of values in each dimension to sample
            %from
            %Q ~ V/N*sum(H)
            H = 0;
            converged = false;
            %%%% Default parameters %%%%%
            dynamic_plot = false;
            static_plot = false;
            check_overlaps = false;
            %start with 1000 points
            npts = 10000;
            pts = (range(:,2)-range(:,1))*ones(1,npts).*rand(size(range,1),npts)...
                +range(:,1)*ones(1,npts);
            
            %set up overlap matrix
            if nargout > 1
                %test chck checkOcc
                testout = checkOcc(range(:,1));
                overlaps = zeros(numel(testout));
                check_overlaps = true;
            end
            
            if nargin < 3
                options = struct();
                
            end
            %set up plot options
            if isfield(options, 'plot')
                if strcmp(options.plot,'dynamic')
                    dynamic_plot = true;
                elseif strcmp(options.plot,'static')
                    static_plot = true;
                end
                
            end
            if isfield(options,'focus')
                focus = options.focus;
            else
                focus = 0;
            end
            
            
            
            for i = 1:npts
                occ = checkOcc(pts(:,i));
                occupied = any(occ);
                if occupied
                    H = H + 1;
                    %mark overlapping ellipses
                    if check_overlaps
                        ellipses = find(occ);
                        %uncount double counted volume
                            %if we're trying to find the volume of a
                            %single ellipse and none of the ellipses
                            %are that one, the point doesn't count
                            %towards volume
                        if focus > 0 && all(ellipses - focus)
                            H = H - 1;
                        end
                        %this double loop assume there will not be a
                        %huge number of ellipses in the same place
                        for j = 1:numel(ellipses)
                            for k = 1:numel(ellipses)
                                overlaps(ellipses(j),ellipses(k)) = 1;
                            end
                        end
                    end
                end
                
                
                
                %plot for debugging
                if (dynamic_plot || static_plot)
                    h = gcf; hold on;
                    
                    if occupied
                        if check_overlaps && numel(find(occ))>1
                            plot(pts(1,i),pts(2,i),'bx');
                        else
                            plot(pts(1,i),pts(2,i),'gx');
                        end
                    else
                        plot(pts(1,i),pts(2,i),'rx');
                    end
                    if dynamic_plot
                        title(sprintf('H %d',H));
                        drawnow;
                    end
                end
            end
            
            v = 1;
            
            for j = 1:size(range,1)
                v = v*(range(j,2)-range(j,1));
            end
            %check for normalization
            if isfield(options, 'normalize')
                if options.normalize
                    v = 1;
                end
            end
            %total volume
            Q = v/npts*H;
            %convergence conditions - need to actually math this:
            % net change after adding 10 points is less than 1%
            
            %sample
            
        end
    end
=======
            
        end
    
        methods (Static)
            
            %montecarlo does a naive montecarlo integration
            function [Q,overlaps] = monteCarlo(checkOcc,range,options)
                %@param checkOcc - function handle that takes a point and
                %returns whether it's occupied or not
                %@param range - range of values in each dimension to sample
                %from
                %Q ~ V/N*sum(H) 
                H = 0;
                converged = false;
                %%%% Default parameters %%%%%
                dynamic_plot = false;
                static_plot = false;
                check_overlaps = false;
                %start with 1000 points
                npts = 1000;
                pts = (range(:,2)-range(:,1))*ones(1,npts).*rand(size(range,1),npts)...
                    +range(:,1)*ones(1,npts);
                
                %set up overlap matrix
                if nargout > 1
                    %test chck checkOcc
                    testout = checkOcc(range(:,1));
                    overlaps = sparse(zeros(numel(testout)));
                    check_overlaps = true;
                end
                
                if nargin < 3 
                    options = struct();
                    
                end
                %set up plot options
                if isfield(options, 'plot')
                    if strcmp(options.plot,'dynamic')
                        dynamic_plot = true;
                    elseif strcmp(options.plot,'static')
                        static_plot = true;
                    end
                    
                end
                if isfield(options,'focus')
                    focus = options.focus;
                else 
                    focus = 0;
                end
            
                for i = 1:npts
                    occ = checkOcc(pts(:,i));
                    occupied = any(occ);
                    if occupied
                        H = H + 1;
                      %mark overlapping ellipses
                        if check_overlaps
                            ellipses = find(occ);
                            %uncount double counted volume
                            if numel(ellipses) > 1 
                                H = H - 1;
                                %if we're trying to find the volume of a
                                %single ellipse and none of the ellipses
                                %are that one, the point doesn't count
                                %towards volume
                            elseif focus > 0 && all(ellipses - focus)
                                H = H - 1;
                            end
                            %this double loop assume there will not be a
                            %huge number of ellipses in the same place
                            for j = 1:numel(ellipses)
                                for k = 1:numel(ellipses)
                                    overlaps(ellipses(j),ellipses(k)) = 1;
                                end
                            end
                        end
                    end
                            
    
                   
                    %plot for debugging
                    if (dynamic_plot || static_plot)
                        h = sfigure(1337); hold on;
                        
                        if occupied 
                            if check_overlaps && numel(find(occ))>1
                                plot(pts(1,i),pts(2,i),'bx');
                            else
                            plot(pts(1,i),pts(2,i),'gx');
                            end
                        else
                            plot(pts(1,i),pts(2,i),'rx');
                        end
                        if dynamic_plot
                            title(sprintf('H %d',H));
                            drawnow;
                        end
                    end
                end
                
                v = 1;
                
                for j = 1:size(range,1)
                    v = v*(range(j,2)-range(j,1));
                end
                Q = v/npts*H;
                %convergence conditions - need to actually math this:
                % net change after adding 10 points is less than 1% 
                
               %sample
               
            end
        end
>>>>>>> 5f12e3960ed647f1e5ace7049fc85481b8c48de0
end