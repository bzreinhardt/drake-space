classdef LinkControlRegion < DrakeSystem
    properties
        c; %controller
        V; % polynomial defining region of attraction 
        neighbors; % neighbors
        x0; %center of the region. Assumes all control ellipses are centered at 0.
        u0; %nominal control of the region - 0 in space
        Q;
        R;
        frame; 
        id; %unique id
        scales;  %scales of each dimension in state space
        sys;
        x; %mss poly of state variables for testing
        bb; %bounding box of the control system, for easier error checking
        
    end
    
    methods
        %Constructor
        function obj = LinkControlRegion(x0,sys)
%should check for x0 matching sys dimensions
            if nargin < 1
                num_states = numel(x0);
                num_inputs = 0;
            else
                num_states = sys.getNumStates;
                num_inputs = sys.getNumInputs;
            end
            obj = obj@DrakeSystem(num_states,0,num_inputs, num_states, false, true);
            obj.u0 = zeros(num_inputs,1);
            obj.Q = blkdiag(10*eye(num_states/2),eye(num_states/2));
            obj.R = eye(num_inputs);
            
            if (nargin > 0), obj.x0 = x0; end;
            if (nargin > 1)
                obj.sys = sys; 
                obj = obj.setStateFrame(sys.getStateFrame);
                obj = obj.setInputFrame(sys.getInputFrame);
                
            end;
            
            tau = sys.equilibriumTorque(x0(sys.getNumStates/2));
            if any(isnan(tau))
                warning('LINKCONTROLREGION: x0 is not a fixed point');
            else
                
                if ~isequal(xstar.double,x0)
                    warning('LINKCONTROLREGION: x0 not a stable point. switch to stable point');
                    disp('new x0 = ');
                    disp(xstar.double);
                    
                end
                obj.x0 = xstar.double;
                obj.u0 = ustar.double;
                disp('u0 =');
                disp(ustar.double);
            end
 
        end
        
        function data = write(obj)
            %output a struct representation of the object
            data.x0 = obj.x0;
        end
        %
        function addPoints(obj,occ)
            %@param occ - sparse occupancy matrix of stabalizable points in
            %state space
            % ADDPOINTS adds all the points in the region to occ
            
            %generate points on the implicit grid that are inside V
        end
        
%         function fr = getFrame(obj)
%             fr = obj.frame;
%         end
%         function u = getNomInput(obj)
%             u = obj.u0;
%         end
        
        %checkForNeighbors and add them to the set of neighbors
        function checkForNeighbors(obj)
            %
            %
        end
        
        %generate a controller and the associated stable region
        function obj = generateController(obj,options)
            if nargin < 2
                options = struct();
                options.method = 'tilqr';
            end
            if strcmp(options.method,'tilqr')
                if ~isa(obj.x0,'Point')
                    x = Point(obj.getStateFrame,obj.x0);
                else x = obj.x0;
                end
                if ~isa(obj.u0,'Point')
                    u = Point(obj.getInputFrame,obj.u0);
                else u = obj.u0;
                end
                [obj.c, obj.V] = tilqr(obj.sys, x, u, obj.Q, obj.R);
            
            %this is mostly meant for testing
            elseif strcmp(options.method,'sphere2d')
                state = obj.getStateFrame.getPoly; 
                obj.x = state(1:2);
                obj.V = sum((obj.x).^2); 
            elseif strcmp(options.method,'sphere6d')
                state = obj.getStateFrame.getPoly;
                obj.x = state(1:6);
                obj.V = sum((obj.x).^2);
            end
            obj.bb = obj.getBoundingBox;
            
        end
        
        %check if a point is in the ball
        function checkPoint(obj,x)
            
        end
        
        %return a function handle that calculates the control ellipse
        function func = getFn(obj, str_output,add_center)
            %@str_output boolean - nonexistant or false: anon fun output
            if nargin < 2
                str_output = false;
            end
            if nargin < 3
                add_center = false;
            end
                
            if isa(obj.V,'LyapunovFunction')
                % 1/16 test using dmsubs instead of fn
               % func = fn(obj.V.getPoly(0),obj.V.getFrame.getPoly);
               
               %dmsubs is still way slower
               %func = @(state)(full(dmsubs(obj.V.getPoly(0),obj.V.getFrame.getPoly,state)));
               [x,p,M,sz]=decomp(obj.V.getPoly(0));
               
               func = decomp2fun(M,p,1,str_output);
            elseif isa(obj.V,'msspoly')
                %func = fn(obj.V,obj.x);
                %func = @(state)(full(dmsubs(obj.V,obj.x,state)));
                %[x,p,M,sz]=decomp(obj.V.getPoly(0));
                [x,p,M,sz]=decomp(obj.V);
                
                func = decomp2fun(M,p,1,str_output);
            else
                error('ControlRegion:c has no controller. Try generateController');
            end
            
            if add_center
                newfunc = '';
                k = 1;
                for j = 1:length(func)
                    if strcmp(func(j),'x')
                        i = str2double(func(j+2));
                        left = func(k:j-1);
                        mid = func(j:j+3);
                        
                        
                        newfunc = strcat(newfunc,left,'(',mid,...
                            sprintf('-%f)',obj.x0(i)));
                        k = j+4;
                    end
                    
                end
                if k <= length(func)
                   newfunc = strcat(newfunc,func(k:end));
                end
                func = newfunc;
            end
        end
        
        %draw the region
        function draw(obj,options)
            %get the projections in the xy plane
            if nargin < 2
                options = struct();
                options.dims = [1,2];
                color = 'g';
                draw_center = true;
            else
                if ~isfield(options,'draw_center')
                    draw_center = true;
                else 
                    draw_center = options.draw_center;
                end
                
                if isfield(options,'color')
                        color = options.color;      
                else
                    color = 'g';
                end
                if isfield(options,'edge_color')
                    edge_color = color;
                else
                    edge_color = 'k';
                end
                
            end
            if isa(obj.V,'LyapunovFunction')
                y = getProjection(obj.V, 0,zeros(size(obj.x0)),options.dims);
            elseif isa(obj.V,'msspoly')
                y = getProjection(obj.x,obj.V,zeros(size(obj.x)),options.dims);
            elseif isempty(obj.V)
                gcf; hold on;
                plot(obj.x0(options.dims(1)),obj.x0(options.dims(2)),'x',...
                    'Color',color);
                xlabel(sprintf('x%d',options.dims(1)));
                ylabel(sprintf('x%d',options.dims(2)));
                hold off;
                return;
            end
            gcf; hold on;
            y = y + obj.x0(options.dims)*ones(1,size(y,2));
            h = fill3(y(1,:),y(2,:),repmat(0,1,size(y,2)),color,...
                'LineStyle','-','LineWidth',2,'EdgeColor',edge_color);
           
            xlabel(sprintf('x%d',options.dims(1)));
            ylabel(sprintf('x%d',options.dims(2)));
            if draw_center
                plot(obj.x0(options.dims(1)),obj.x0(options.dims(2)),'x','Color',color,'MarkerSize',10);
            end
            hold off;
        end
        
        %return the bounding box on the region
        function range = getBoundingBox(obj)
            if isa(obj.V,'LyapunovFunction')
                y = getLevelSet(obj.V)+obj.x0*ones(1,100);
            elseif isa(obj.V, 'msspoly')
                y = getLevelSet(obj.x,obj.V)+obj.x0*ones(1,100);
            end
            range = [min(y,[],2),max(y,[],2)];
        end
        
            
        
        %find grid points inside level set
        function pointsInLevelset(obj,V)
            %@param V - an mss poly representing the area in question
            %must be like lyapunov function > 0 everywhere
            % for now assume everything V<= 1 is stabalizable
            lvl_set = V.getLevelSet();
         
            min_max = [min(lvl_set,2),max(lvl_set,2)];
            ranges = cell(size(obj,scales));
           
            for i = 1:length(obj.scale)
                %vector of points
                ranges{i} = min_max(i,1):obj.scale:min_max(i,2);
            end
            points = cell(size(ranges));
            [points{:}] = ndgrid(ranges{:});
            
        end
    end
        
        methods (Static)
            
            
        end
end