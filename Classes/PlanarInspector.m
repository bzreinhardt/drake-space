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
            obj.setInputLimits(-1000,1000);
            
            obj.bounding_sphere = max(sqrt(sum(obj.d.^2,2)));
            load('coupler_splines.mat');
            obj.fx_spline = f_x_spline;
            obj.fy_spline = f_y_spline;
            
        end
        
        function xcdot = dynamics(obj,t,X,u)
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
            x0 = [0;0.1;0;0;0;0];
            %prog is the constraint function, bounding box constraint
            %binding the system to be between x0 and x0 at the first knot
            prog = addStateConstraint(prog,BoundingBoxConstraint(x0,x0),1);
            
            %add the final value constraint
            xf = [0;0.11;0;0;0;0];
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

