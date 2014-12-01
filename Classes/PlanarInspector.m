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
        function obj = PlanarInspector()
            num_xc = 6;
            num_xd = 0;
            num_u = 2;
            obj@DrakeSystem(num_xc,num_xd,num_u,num_xc,false,true);
            %set up couplers
            d1 = 0.1*[1/2^.5,-1/2^.5,0];
            d2 = 0.1*[-1/2^.5,-1/2^.5,0];
            a1 = [0,0,1];
            a2 = [0,0,1];
            
            obj.a = [a1;a2];
            obj.d = [d1;d2];
            
            %constrained inputs
            %obj.setInputLimits(-1000,1000);
            
            obj.bounding_sphere = max(sqrt(sum(obj.d.^2,2)));
            load('coupler_splines.mat');
            obj.fx_spline = f_x_spline;
            obj.fy_spline = f_y_spline;
            
        end
        
        function xcdot = oldDynamics(obj,t,X,u)
            x = X(1,:)';
            y = X(2,:)';
            theta = X(3,:)';
            vx = X(4,:)';
            vy = X(5,:)';
            omega = X(6,:)';
            u = u';
            
            % TODO add in velocity dynamics - ignore for now
            vx_plate = zeros(size(vx));
            vy_plate = zeros(size(vy));
            
            
            net_force_x = zeros(size(vx));
            net_force_y = zeros(size(vy));
            net_torque = zeros(size(omega));
            %cycle through the forces and torques from each coupler
            
            for i = 1:size(obj.a,1)
                %find coupler positions in world coordinates
                coupler_x = obj.d(i,1)*cos(theta)-obj.d(i,2)*sin(theta)+x;
                coupler_y = obj.d(i,1)*sin(theta)+obj.d(i,2)*cos(theta)+y;
                
                [g,surf_norm] = obj.sphereNormGap([coupler_x,coupler_y]);
                
                if any(any(isnan([g,vx_plate,vy_plate,u(:,i)])))
                    fx = NaN*net_force_x;
                    fy = NaN*net_force_y;
                else
                    fx = fnval(obj.fx_spline,[vx_plate,vy_plate,g,u(:,i)]')';
                    fy = fnval(obj.fy_spline,[vx_plate,vy_plate,g,u(:,i)]')';
                end
                
                force_x = -surf_norm(:,2).*fx + surf_norm(:,1).*fy;
                force_y = surf_norm(:,1).*fx + surf_norm(:,2).*fy;
                torque = -(obj.d(i,1)*sin(theta)+obj.d(i,2)*cos(theta)).*force_x + ...
                    (obj.d(i,1)*cos(theta)-obj.d(i,2)*sin(theta)).*force_y;
                
                net_force_x = net_force_x + force_x;
                net_force_y = net_force_y + force_y;
                net_torque = net_torque + torque;
            end
            
            xcdot = zeros(size(X));
            xcdot(1,:) = vx';
            xcdot(2,:) = vy';
            xcdot(3,:) = omega';
            xcdot(4,:) = net_force_x';
            xcdot(5,:) = net_force_y';
            xcdot(6,:) = net_torque';
            if ~isequal(size(xcdot),[6,1])
                disp('xcdot size ');
                disp(size(xcdot));
            end
        end
        %Attempt at drake gradientalizable dynamics
        
        function [xcdot, dxcdot] = simpleDynamics(obj,t,X,u)
             x = X(1);
            y = X(2);
            theta = X(3);
            vx = X(4);
            vy = X(5);
            omega = X(6);
            u1 = u(1);
            u2 = u(2);
            
            d = [0,-0.1];
            
            R_wb = [cos(theta) -sin(theta); sin(theta) cos(theta)];
            
             net_force = [u1];
            
            net_torque = 0;
            
            function df = findGradient(obj,t,X,u)
            df = sparse(size(X,1),size(X,1)+size(t,1)+size(u,1));
            df(1,5) = 1;
            df(2,6) = 1;
            df(3,7) = 1;
            df(4,8) = 1;
            df(5,9) = 1;
            end
        
      
            
             xcdot = [vx;vy;omega;net_force;net_torque];
            dxcdot = findGradient(obj,t,X,u);
        end
            
        function [xcdot, dxcdot] = dynamics(obj,t,X,u)
          
            
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
                g = norm(d_w - obj.sphere_center)-obj.sphere_radius;
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
            dxcdot = findGradient(obj,t,X,u);
            
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
        
    end
    methods (Static = true)
        function runDircol
            plant = PlanarInspector;
            
            N = 20;
            %[0.1 10] are bounds on the duration
            prog = DircolTrajectoryOptimization(plant,N,[0.1 10]);
            
            %add initial value constraint
            x0 = [0;0.2;0;0;0;0];
            %prog is the constraint function, bounding box constraint
            %binding the system to be between x0 and x0 at the first knot
            prog = addStateConstraint(prog,BoundingBoxConstraint(x0,x0),1);
            
            %add the final value constraint
            xf = [0;0.2;0;0;0;0];
            prog = addStateConstraint(prog,BoundingBoxConstraint(xf,xf),N);
            
            %add the cost function g(dt,x,u) = 1*dt - cost is only on time
            
            % add the cost function g(dt,x,u) = 1*dt
            function [g,dg] = cost(dt,x,u)
                g = dt; dg = [1,0*x',0*u']; % see geval.m for our gradient format
            end
            prog = addRunningCost(prog,@cost);
            
            % add a display function to draw the trajectory on every iteration
            function displayStateTrajectory(t,x,u)
                plot(x(1,:),x(2,:),'b.-','MarkerSize',10);
                axis([-5,1,-2.5,2.5]); axis equal;
                drawnow;
            end
            prog = addTrajectoryDisplayFunction(prog,@displayStateTrajectory);
            
            [xtraj,	utraj, z,F,	info, infeasible_constraint_name] = prog.solveTraj(2);
        end
        
        function runOL
            
        end
    end
    
end

