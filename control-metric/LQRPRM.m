classdef LQRPRM < HybridDrakeSystem, 
    %LQRPRM holds the controllers the define a global control policy
    properties
        occupancy_map;
        volume; %normalized stabilizable volume in state space
        scales; %scales of each dimension in state space
        regions; %array of regional controllers
        tot_range; %range of each dimension in state space for generating controllers
        sys;
        regions_max; %maximum number of regions in the system
        volume_update = true;
        
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
                obj.tot_range = [-0.1 0.1; 0.09 0.2; ...
                    -pi/2 pi/2; -0.1 0.1; ...
                    -0.1 0.1; -0.01 0.01];
            elseif sys.getNumStates == 12
            end
            %�?�?�????���??��?�????�?�?�
            obj.sys = sys;
            obj.regions = {};
            obj.occupancy_map = sparse(0);
            obj.volume = 0;
            obj.regions_max = 15;
            
        end
        
        %populate the LQRPRM with controllers
        function obj = fillRegion(obj,options)
            
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
                
               obj = obj.findVolume(options);
            end
            
            %prune controllers
        end
        %% 
        
        function vol = getVolume(obj)
        
            vol = obj.volume;
                
            
        end
        function obj = findVolume(obj, options)
            if nargin < 2
                options = struct();
            end
            if isfield(options,'slow_occ_test')
            %occupancy test to check against each ellipse
                occ_test = cell(numel(obj.regions),1);
                for i = 1:numel(obj.regions)
                    func = obj.regions{i}.getFn; %get the lyapunov polynomial
                    occ_test{i} = @(state)(func(state-obj.regions{i}.x0) <= 1); %see whether state is within level set
                end
                checkOcc = @(state)(cellfun(@(func)(func(state)),occ_test));
            else
                checkOcc = obj.buildFastOccTest();
            end
                [Q,overlaps] = obj.monteCarlo(checkOcc,obj.getBoundingBox,options);
                obj.volume = Q;
                obj.occupancy_map = overlaps;
                obj.volume_update = false;
                
            
        end
        
        function func = buildFastOccTest(obj,options)
            if nargin < 2 
                options = struct();
            end
            occ_test = '@(x)([';
            for i = 1:numel(obj.regions)
                fn = obj.regions{i}.getFn(true,true);
                occ_test = strcat(occ_test,fn,';');

            end
            occ_test = strcat(occ_test,'] <= [');
            for i = 1:numel(obj.regions)
                occ_test = strcat(occ_test,'1;');
            end
            occ_test = strcat(occ_test,'])');
            func = str2func(occ_test);
        end
        
        function obj = fastFindVolume(obj,options)
            error('FASTFINDVOLUME:ERROR NOT IMPLEMENTED YET');
%             if nargin < 2
%                 options = struct();
%             end
%             fns = cell(numel(obj.regions),1);
%             for i = 1:numel(obj.regions)
%                 str_fn = obj.regions{i}.getFn(true);
%                 fns{i} = strcat('(',str_fn,')'
%                 % need to pull the regions center into the function
%             end
        end
        
        function obj = genControlRegion(obj,options)
            %generate a random ellipse
            if nargin < 1
                options = struct();
                options.method = 'sphere2d';
                options.random = 'true';
                options.connect_at_end = 'true';
            end
            %generate the cener of the control region
            if isfield(options,'x0')
                x0 = options.x0;
            else
                %pick a random box center (use heuristic here')
                x0 = obj.generateCenter();
            end
            if ~isfield(options,'connect_at_end')
                options.connect_at_end = 'true';
            end
            c = ControlRegion(x0,obj.sys);
            c = c.generateController(options);
            obj.regions{1+numel(obj.regions)} = c;
            %debugging drawing
            if ~options.connect_at_end
                obj = obj.connectControlRegion(c,options);
            end
        end
        %generate a new point for a control center
        function x0 = generateCenter(obj)
            x0 = (obj.tot_range(:,2)-obj.tot_range(:,1)).*rand(size(obj.tot_range,1),1)...
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
        
        function stochasticIntegration(obj)
            %generate a point at random in the range
            %check it against each sphere
            
            
        end
        
        function bb = getBoundingBox(obj)
            bb = zeros(obj.sys.getNumStates,2);
            for i = 1:numel(obj.regions)
                c_bb = obj.regions{i}.getBoundingBox();
                %pad bounding box with zeros if controller is lower
                %dimension than the system
                if size(c_bb,1) < obj.sys.getNumStates
                    c_bb = [c_bb; zeros(obj.sys.getNumStates - size(c_bb,1),2)];
                end
                diff_bb = [bb(:,1) - c_bb(:,1), c_bb(:,2) - bb(:,2)];
                ind = find(diff_bb > 0);
                bb(ind) = c_bb(ind);
            end
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
        %% Plotting Functions
        function drawCenter(obj,fig,options)
        end
        
        function draw(obj,fig,options)
            
            if nargin < 2
                fig = gcf;
            end
            if nargin < 3
                options.dims = [1,2];
                options.color = 'g';

            end
            rand_color = false;
            if isfield(options,'color')
                if strcmp(options.color,'rand')
                    rand_color = true;
                end
            end
              
                
       
            
            figure(fig); hold on;
            
            for i = 1:numel(obj.regions)
                
                if rand_color
                        cc=hsv(12);
                        options.color = cc(mod(i,12),:);
                end
                obj.regions{i}.draw(options);
                
                if isfield(options,'label') 
                    text(obj.regions{i}.x0(options.dims(1)),...
                        obj.regions{i}.x0(options.dims(2)),...
                       sprintf('%d',i),'FontSize',20); 
                end
            end
            alpha(0.05);
            
            %two ways of doing it, the stochastic way and the not
            %stochastic way
            
        end
        %get the bounding box for the entire controller
        
        function plotGraph(obj,fig,options)
            if nargin < 2 
                fig = gcf;
            end
            if nargin < 3
                options.dims = [1,2];
                options.color = 'g';
            end
            figure(fig);hold on;
            for i = 1:numel(obj.regions)
                %assume 2d for now
                plot(obj.regions{i}.x0(options.dims(1)),obj.regions{i}.x0(options.dims(2)),...
                    'o','MarkerSize',10);
                text(obj.regions{i}.x0(options.dims(1)),...
                        obj.regions{i}.x0(options.dims(2)),...
                       sprintf('%d',i),'FontSize',20); 
                for j = 1:numel(obj.regions)
                    if obj.occupancy_map(i,j)
                        plot([obj.regions{i}.x0(options.dims(1)),obj.regions{j}.x0(options.dims(1))],...
                            [obj.regions{i}.x0(options.dims(2)),obj.regions{j}.x0(options.dims(2))]);
                    end
                end
            end
        end
        
        function save(obj,filename,notes)
            %write the PRM to a file
            date_string = datestr(now);
         if nargin < 2 %autogena filename
             date_string((ismember(date_string,' ') == 1)) = '-';
             date_string((ismember(date_string,':') == 1)) = '_';
             filename = strcat('lqrprm_data_',date_string,'.json');
         end
         %generate a header
         data = obj.write;
         data.notes = notes;
         savejson('',data,'FileName',filename);
        end
     
        function data = write(obj)
            %output a struct representing the object
            
            data.controller_data = cell(numel(obj.regions),1);
            for i = 1:numel(obj.regions)
                data.controller_data{i} = obj.regions{i}.write();
            end
           
            
            data.sys_data = obj.sys.write();
            
            data.volume = obj.volume;
            data.map = obj.occupancy_map;
           
        end
        
        
    end
    
    methods (Static)
        
        %montecarlo does a naive montecarlo integration
        function [Q,overlaps] = monteCarlo(checkOcc,range,options)
            %@param checkOcc - function handle that takes a point and
            %returns whether it's occupied or not
            %@param range - range of values in each dimension to sample
            %from
            %Q ~ V/N*sum(H)
            
            %%%%%%% Defualt options
            focus = 0;
            dynamic_plot = false;
                static_plot = false;
                rand_method = 'halton';
                            npts = 5000;
                            normalize = false;
            
            % parse inputs
            
            if nargin < 3
                options = struct();
                
            end
            if isfield(options,'focus')
                focus = options.focus;
      
            end
            if isfield(options, 'plot')
                if strcmp(options.plot,'dynamic')
                    dynamic_plot = true;
                elseif strcmp(options.plot,'static')
                    static_plot = true;
                end
                
            end
            if isfield(options,'rand_method')
                rand_method = options.rand_method;
                
            end
            if isfield(options,'npts')
                npts = options.npts;
            end
            
             if isfield(options, 'normalize')
               normalize = true;
            end
            
            if nargout > 1
                %test chck checkOcc
                testout = checkOcc(range(:,1));
                overlaps = zeros(numel(testout));
                check_overlaps = true;
            else
                check_overlaps = false;
     
            end
            
           
            
            pts = genRandPts(npts,size(range,1),range,rand_method);

            H = 0;

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
            if ~normalize
                for j = 1:size(range,1)
                    if (range(j,2)-range(j,1)) > 0
                    v = v*(range(j,2)-range(j,1));
                    end
                end
            end
            
            %total volume
           Q = v/npts*H;
            %convergence conditions - need to actually math this:
            % net change after adding 10 points is less than 1%
            
            %sample
            
        end
    end
end