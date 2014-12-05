old function xcdot = oldDynamics(obj,t,X,u)
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