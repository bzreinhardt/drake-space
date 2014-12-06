classdef PlanarInspector < DrakeSystem
    %Planar dynamics of a two coupler induction inspector
    
    properties
        sphere_radius = 5;
        sphere_center = [0;-5];
        fx_spline; %coupler x force spline
        fy_spline; %coupler y force spline
        d; %coupler positions in body coordinates
        a; %coupler axes in body coordinates
        bounding_sphere;
        % These coefficients come from a fit in PolyForceFits.m. They are
        % reasonably good fits for medium-sized induction couplers
%         f_x = @(u,g)(-5.6219e-08.*u.^3+6.2003e-06.*u.^2+76.0622.*u).*6.2003e-06.*exp(-44.1463.*g)
%         f_y = @(u,g)(5.811e-09.*u.^4+-2.2676e-08.*u.^3+-1.8637.*u.^2+0.29795.*u).*-33.8115.*exp(-38.9901.*g+-21.8594)
    end
    
    methods
        %constructor
        function obj = PlanarInspector(a,d)
            %set up couplers
            if nargin < 2
            d1 = 0.1*[1/2^.5,-1/2^.5,0];
            d2 = 0.1*[-1/2^.5,-1/2^.5,0];
            
            d = [d1;d2];
            end
            if nargin < 1
            a1 = [0,0,1];
            a2 = [0,0,1];
            a = [a1;a2];
            end
            num_xc = 6;
            num_xd = 0;
            num_u = size(a,1);
            obj@DrakeSystem(num_xc,num_xd,num_u,num_xc,false,true);
            obj.a = a;
            obj.d = d;
            
            
            
            
            %constrained inputs
            obj= obj.setInputLimits(-5000,5000);
            
            obj.bounding_sphere = max(sqrt(sum(obj.d.^2,2)));
%             load('coupler_splines.mat');
%             obj.fx_spline = f_x_spline;
%             obj.fy_spline = f_y_spline;
            obj = obj.setOutputFrame(obj.getStateFrame);
            
        end
        
         function [xcdot, dxcdot] = dynamics(obj,t,X,u)
             [xcdot] = drakeifiedPlanarDynamics(obj,t,X,u);
             [dxcdot] = planarJacobian(X(3),u(1),u(2),u(3),X(1),X(2));
         end
         
         function generateGradients(obj)
             filename = 'planarJacobian.m';
             generateInspectorGradients(obj,filename);
         end
             
            
        function [xcdot, dxcdot] = old_dynamics(obj,t,X,u)
          
            x = X(1);
            y = X(2);
            theta = X(3);
            vx = X(4);
            vy = X(5);
            omega = X(6);
            
            R_wb = [cos(theta) -sin(theta); sin(theta) cos(theta)];
            
            net_force = zeros(2,1);
            
            net_torque = 0;
            %cycle through the forces and torques from each coupler
            
            for i = 1:size(obj.d,1)
                %find coupler positions in world coordinates
                % coupler_x = obj.d(i,1)*cos(theta)-obj.d(i,2)*sin(theta)+x;
                % coupler_y = obj.d(i,1)*sin(theta)+obj.d(i,2)*cos(theta)+y;
                %Coupler coordinates in world frame
                d_w = R_wb*obj.d(i,1:2)' + [x;y];
                %gap between sphere and coupler
                try
                g = norm(d_w - obj.sphere_center)-obj.sphere_radius;
                catch
                    disp('norm not working');
                end
                %normal at nearest point
                n = (d_w - obj.sphere_radius)/norm(d_w - obj.sphere_radius);
                %rotation to take world coordinates into plate coordinates
                R_pw = [-n(2) n(1); n(1) n(2)];
                
                %v_plate = R_pw*[vx;vy];
                fx = (-5.6219e-08.*u(i).^3+6.2003e-06.*u(i).^2+76.0622.*u(i))...
                    .*6.2003e-06.*exp(-44.1463.*g);
                fy = (5.811e-09.*u(i).^4+-2.2676e-08.*u(i).^3+-1.8637...
                    .*u(i).^2+0.29795.*u(i)).*-33.8115.*exp(-38.9901.*g+-21.8594);
               
                
                force = R_pw'*[fx;fy];
                torque = -d_w(2)*force(1) + d_w(1)*force(2);
                
                
                net_force = net_force + force;
                net_torque = net_torque + torque;
            end
            
            xcdot = [vx;vy;omega;net_force;net_torque];
%             function df = findGradient(obj,t,X,u)
%                 
%             end
            %dxcdot = planarGradients(obj,t,X,u,1);
            function df = findGradient(obj,t,X,u)
            df = sparse(size(X,1),size(X,1)+size(t,1)+size(u,1));
            df(1,5) = 1;
            df(2,6) = 1;
            df(3,7) = 1;
            df(4,8) = 1;
            df(5,9) = 1;
            end
            %dxcdot = findGradient(obj,t,X,u);
            %dxcdot = planarInspectorGradients(obj,t,X,u,1);
            dxcdot = ones(obj.getNumStates,1+obj.getNumStates+obj.getNumInputs);
            
        end
        
        function [g, surf_norm] = sphereNormGap(obj,x)
            % Find the gap between an actuator and the closest point on a sphere and
            % its normal
            %TODO vectorize
            % @param x Nx3 position in world coordinates
            
            states = 3;
            if (size(x,2) == 2)
                %assume you're in 2D
                states = 2;
                c = obj.sphere_center(1:states);
            end
            
            R = x(:,1:states)-ones(size(x,1),1)*c';
            surf_norm = R./(sqrt(sum(R.^2,2))*ones(1,states));
            
            g = sqrt(sum(R.^2,2))-obj.sphere_radius;
            if any(g<=0)
                error_msg = strcat('sphereNormGap: coupler is inside the freaking sphere');
                warning(error_msg);
            end
        end
        
        function y = output(obj,t,x,u)
            y = x;
        end
        
        function [c,V] = findLQR(obj,x0,u0)
            %Generates an lqr controller for the inspector around a given
            %point
            x0 = Point(obj.getStateFrame,x0);
            if nargin < 3
            u0 = Point(obj.getInputFrame,zeros(getNumInputs(obj),1));
            else
                u0 = Point(obj.getInputFrame,u0);
            end
            Q = diag([100000000 100000000 1000000 0.1 0.1 0.1]);
            R = eye(obj.getNumInputs);
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
        
    end
    methods (Static = true)
       
        
        
        
        
        
    end
    
end

