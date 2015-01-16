classdef Inspector2d < DrakeSystem
    %Planar dynamics of an induction nspector
    
    properties
        % properties of the sphere to maneuver around
        sphere_radius = 5;
        sphere_center = [0;-5];
       
        d; %coupler positions in body coordinates
        a; %coupler axes in body coordinates
        bounding_sphere;
        couplers; % couplers on the inspector
        % These coefficients come from a fit in PolyForceFits.m. They are
        % reasonably good fits for medium-sized induction couplers
%         f_x = @(u,g)(-5.6219e-08.*u.^3+6.2003e-06.*u.^2+76.0622.*u).*6.2003e-06.*exp(-44.1463.*g)
%         f_y = @(u,g)(5.811e-09.*u.^4+-2.2676e-08.*u.^3+-1.8637.*u.^2+0.29795.*u).*-33.8115.*exp(-38.9901.*g+-21.8594)
    end
    
    methods
        %constructor
        function obj = Inspector2d(a,d)
            %set up couplers
            %n = number of couplers
            if nargin < 2
            d1 = 0.12*[1/2^.5;-1/2^.5;0];
            d2 = 0.12*[-1/2^.5;-1/2^.5;0];
            d3 = 0.07*[0; -1; 0];
            d = [d1,d2,d3];
            end
            if nargin < 1
            a1 = [0;0;1];
            a2 = [0;0;1];
            a3 = [0;0;1];
            a = [a1,a2,a3];
            end
            %6 continuous states
            num_xc = 6;
            % no discrete states
            num_xd = 0;
            num_u = size(a,2);
            obj@DrakeSystem(num_xc,num_xd,num_u,num_xc,false,true);
            obj.a = a;
            obj.d = d;
            
            %constrained inputs
            obj= obj.setInputLimits(-10000,10000);
            
            obj.bounding_sphere = max(sqrt(sum(obj.d.^2,2)));
            %full state feedback
            obj = obj.setOutputFrame(obj.getStateFrame);
        end
        
         function [xcdot] = dynamics(obj,t,X,u)
             xcdot = [X(size(X,1)/2+1:end);zeros(size(X,1)/2,1)];
              if nargout > 1
                dxcdot = sparse(size(X,1),size(t,1)+size(X,1)+size(u,1));
                dxcdot(1:size(X,1)/2,2+size(X,1)/2:1+size(X,1)) = eye(size(X,1)/2);
             end
             options.radius = obj.sphere_radius;
             options.center = obj.sphere_center;
             
             for i = 1:obj.num_u
                 options.d = obj.d(:,i);
                 %find coupler in global frame 
                 xcdot = xcdot+couplerDynamics2D(t,X,u(i),options);
                 if nargout > 1
                     coupler_grad = couplerGradient2D(t,X,u(i),options);
                     grad_add = [coupler_grad(:,1:1+obj.getNumStates),...
                         zeros(size(coupler_grad,1),obj.getNumInputs)];
                     grad_add(:,(1+obj.getNumStates+i)) = coupler_grad(:,end);
                 dxcdot = dxcdot + grad_add;
                 end
             end
         end 
        
        function y = output(obj,t,x,u)
            y = x;
        end
        
        function [c,V0] = findLQR(obj,x0,u0)
            %Generates an lqr controller for the inspector around a given
            %point
            x0 = Point(obj.getStateFrame,x0);
            if nargin < 3
            u0 = Point(obj.getInputFrame,zeros(getNumInputs(obj),1));
            else
                u0 = Point(obj.getInputFrame,u0);
            end
            Q = diag([1000 1000 1000 0.1 0.1 0.1]);
            R = 0.00001*eye(obj.getNumInputs);
            if (nargout>1)
                [c,V0] = tilqr(obj,x0,u0,Q,R);
                sys = feedback(obj,c);
                
                pp = sys.taylorApprox(0,x0,[],3);  % make polynomial approximation
                options=struct();
                options.degL1=2;
                %options.method='bilinear';
                %options.degV=4;
                V=regionOfAttraction(pp,V0,options);
            else
                c = tilqr(obj,x0,u0,Q,R);
            end
        end
        
        function [c,V] = findTVLQR(obj,xtraj,utraj)
            %Generates an lqr controller for the inspector around a given
            %point
            xtraj = setOutputFrame(xtraj,getStateFrame(obj));
            utraj = setOutputFrame(utraj,getInputFrame(obj));
            Q = diag([100000000 100000000 100000 1 1 0.1]);
            R = eye(obj.getNumInputs);
            Qf=0.001*diag([(1/0.05)^2 (1/0.05)^2 (1/3)^2 (1/3)^2 1 (1/3)^2]);
            c = tvlqr(obj,xtraj,utraj,Q,R,Qf);
        end
        
        
        function [ytraj,xtraj] = runOL(obj,utraj,x0)
            
            v = PlanarInspectVisualizer(obj);
            if nargin < 2
                x0 = Point(getStateFrame(obj),[0;0.1;0;0;0;0]);
            end
            if nargin < 1
                u0 = Point(getInputFrame(obj),[0.15;-0.15]);
                sys = cascade(ConstantTrajectory(u0),obj);
                tspan = [0 20];
            else
                utraj = utraj.setOutputFrame(getInputFrame(obj));
                sys = cascade(utraj,obj);
                tspan = utraj.getTimeSpan();
            end
            
            [ytraj,xtraj] = simulate(sys,tspan,x0);
            %v.playback(xtraj);
            
        end
        
         function output = runDircol(obj)
       
            t_guess = 30;
            N = 30;
            %[0.1 10] are bounds on the duration
            prog = DircolTrajectoryOptimization(obj,N,[0.1 50]);
            
            %add initial value constraint
            x0 = [0;0.15;0;0;0;0];
            %prog is the constraint function, bounding box constraint
            %binding the system to be between x0 and x0 at the first knot
            prog = addStateConstraint(prog,BoundingBoxConstraint(x0,x0),1);
            
            %add the final value constraint
            xf = [0;0.1;0;0;0;0];
            prog = addStateConstraint(prog,BoundingBoxConstraint(xf,xf),N);
            
            %add the cost function g(dt,x,u) = 1*dt - cost is only on time
            
            % add the cost function g(dt,x,u) = 1*dt
            function [g,dg] = cost(dt,x,u)
                g = dt + u'*u; dg = [1,0*x',2*u']; % see geval.m for our gradient format
            end
            prog = addRunningCost(prog,@cost);
            
            % add a display function to draw the trajectory on every iteration
            function displayStateTrajectory(t,x,u)
                plot(x(1,:),x(2,:),'b.-','MarkerSize',10);
                axis([-0.1,0.1,0,0.3]); axis equal;
                drawnow;
            end
            prog = addTrajectoryDisplayFunction(prog,@displayStateTrajectory);
            tic;
            if nargout > 0
                [xtraj,	utraj, z,F,	info, infeasible_constraint_name] = prog.solveTraj(t_guess);
                output.xtraj = xtraj;
                output.utraj = utraj;
                output.z = z;
                output.F = F;
                output.info = info;
                output.infeasible_constraint_name = infeasible_constraint_name;
            else
                prog.solveTraj(t_guess);
            end
            toc;
            
         end
         %% Set a baseline configuration 
         function obj = setBaseline(obj,type)
             obj.d = [zeros(3,4)];
             switch type
                 case 'four_coupler'
                     theta = [-pi/4, -pi/8, pi/8, pi/4];
                     for i = 1:numel(theta)
                         obj.d(1:2,i) = [cos(theta(i)) -sin(theta(i)); ...
                             sin(theta(i)) cos(theta(i))]*...
                             [0;-0.1];
                     end
                     obj.a = [zeros(2,4); ones(1,4)];
             end
         end
        
        
    end
    methods (Static = true)
       
        
        
        
        
        
    end
    
end

