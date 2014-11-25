classdef PlanarInspectorPlant < SecondOrderSystem

  % state:  
  %  q(1) - x position
  %  q(2) - z position
  %  q(3) - pitch (theta)
  % input:
  %  u(1) - prop 1 thrust
  %  u(2) - prop 2 thrust

  properties  %  based on (Bouadi, Bouchoucha, Tadjine 2007)
    sphere_radius = 5;
        sphere_center = [0;-5];
        fx_spline; %coupler x force spline
        fy_spline; %coupler y force spline
        d; %coupler positions in body coordinates
        a; %coupler axes in body coordinates
        bounding_sphere;
  end
  
  methods
    function obj = PlanarInspectorPlant()
      obj = obj@SecondOrderSystem(3,2,true);
      obj = obj.setOutputFrame(obj.getStateFrame);  % allow full-state feedback
    end
    
    function qdd = sodynamics(obj,t,q,qd,u)
       x = q(1,:)';
            y = q(2,:)';
            theta = q(3,:)';
            vx = qd(1,:)';
            vy = qd(2,:)';
            omega = qd(3,:)';
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
            
            qdd = zeros(size(qd));
           
            qdd(1,:) = net_force_x';
            qdd(2,:) = net_force_y';
            qdd(3,:) = net_torque';
    end
    
    function x = getInitialState(obj)
      x = randn(6,1);
    end
    
    function [c,V] = hoverLQR(obj)
      x0 = Point(obj.getStateFrame,zeros(6,1));
      u0 = Point(obj.getInputFrame,obj.m*obj.g/2 * [1;1]);
      Q = diag([10 10 10 1 1 (obj.L/2/pi)]);  %Q = diag([10*ones(1,3) ones(1,3)]);
      R = [0.1 0.05; 0.05 0.1];  %R = diag([0.1 0.1]);

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
