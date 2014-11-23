classdef RigidBodyCoupler < RigidBodyForceElement

  methods
    function obj = RigidBodyCoupler(frame_id, axis, scale_factor, limits)
     
      obj.kinframe = frame_id;
      obj.axis = axis/norm(axis);      
      
      obj.direct_feedthrough_flag = true; %inputs go through this
      obj.input_num = 0;
      obj.input_limits = [-inf inf];
      if (nargin > 2)
        obj.scale_factor = scale_factor;
      end
      if (nargin > 3)
        obj.input_limits = limits;
      end
      obj.use_bullet = true;
             obj.sigma = 1/obj.rho; %conductivity of the plate 
             obj.g_lims = obj.x_lims.^3;
             obj.C = findC(obj);
      
    end %constructor
%% Compute full nonlinear forces    
    function [force, B_mod, dforce, dB_mod] = computeNonAffineForce(obj,manip,q,qd,u)
        % COMPUTENONAFFINEFORECE(manip,q,qd)
        % @param manip - rigidBodyManipulator experiencing the force
        % @param q - system state
        % @param qd - time derivative of system state
        % B_mod maps the input to generalized forces.

      force = sparse(6,getNumBodies(manip))*q(1); %why multiply by q(1) here?
      
       %Find distance to surface, normal at the surface
      %For now, assume all surfaces are attached to world, which is body 1
      B_mod = manip.B*0*q(1); %initialize B_mod
     
       

      if (nargout>2)  % then compute gradients
          warning('RigidbodyCoupler: gradiennts not implemented yet'); 
        kinsol = doKinematics(manip,q,true);
        [x,J,dJ] = forwardKin(manip,kinsol,obj.kinframe,zeros(3,1)); %body origin in global frame
        [axis_world,Jaxis_world] = forwardKin(manip,kinsol,obj.kinframe,obj.axis); %thrust axis in global frame
        
        daxis_world = Jaxis_world-J; 
        axis_world = axis_world-x;

        dforce = sparse(6*getNumBodies(manip),getNumStates(manip));

        nq = getNumPositions(manip); nu = getNumInputs(manip);
        dB_mod = sparse(nq*nu,getNumStates(manip));
        dB_mod((obj.input_num-1)*nq + (1:nq),1:nq) = obj.scale_factor*(J'*daxis_world + reshape(dJ'*axis_world,nq,nq));
        
      else
        kinsol = doKinematics(manip,q);
        [x,J] = forwardKin(manip,kinsol,obj.kinframe,zeros(3,1));
        axis_world = forwardKin(manip,kinsol,obj.kinframe,obj.axis);
        axis_world = axis_world-x;
      end
      
        % find distance to surface and normal at surface
      %Bullet way
         active_collision_options.body_idx = [obj.parent_link;1];
      [phi,surf_norm,xA,xB,idxA,idxB] = manip.collisionDetect(kinsol, ...
                                           false, ...
                                           active_collision_options);
      
      %find directions for forces
      dir_x = cross(axis_world, surf_norm);
      
      dir_y = cross(dir_x, axis_world);
      if norm(dir_x) == 0 
          warning('spin axis perpendicular to surface');
          v_plate = zeros(3,1);
      else
        y_plate = surf_norm;
        x_plate = cross(axis_world, surf_norm);
        x_plate = x_plate/norm(x_plate);
        z_plate = cross(x_plate,y_plate);
        basis_plate = [x_plate,y_plate,z_plate];
        
        qd_world = J*qd;
        v_world = qd_world(1:3);
        v_plate = basis_plate'*v_world;
        
      end
      
      %convert v_world to v in the surface of the plane
      
     
       %Analytical way
       %TODO
       
       %Find f1 and f2 in the plate frame
       f = obj.findForce(phi,v_plate,u);     
      %force in world frame, scaled based on orientation 
      f_coupler = f(1)*dir_x + f(2)*dir_y;
      % apply force along the z-axis of the reference frame
      B_mod(:,obj.input_num) = J'*f_coupler;
    end
  %% Compute the control affine force 
    function [force, B_mod, dforce, dB_mod] = computeSpatialForce(obj,manip,q,qd)
        % If no input is given, return B linearized around U = 0;
        % COMPUTESPATIALFORCE(manip,q,qd)
        % @param manip - rigidBodyManipulator experiencing the force
        % @param q - system state
        % @param qd - time derivative of system state
        % B_mod maps the input to generalized forces.
     
      force = sparse(6,getNumBodies(manip))*q(1); %why multiply by q(1) here?
      
      B_mod = manip.B*0*q(1); %initialize B_mod
  %Find distance to surface, normal at the surface
      
       
      % apply force along the z-axis of the reference frame
      if (nargout>2)  % then compute gradients
        kinsol = doKinematics(manip,q,true);
        [x,J,dJ] = forwardKin(manip,kinsol,obj.kinframe,zeros(3,1)); %body origin in global frame
        [axis_world,Jaxis_world] = forwardKin(manip,kinsol,obj.kinframe,obj.axis); %thrust axis in global frame
        
        daxis_world = Jaxis_world-J; 
        axis_world = axis_world-x;

        dforce = sparse(6*getNumBodies(manip),getNumStates(manip));

        nq = getNumPositions(manip); nu = getNumInputs(manip);
        dB_mod = sparse(nq*nu,getNumStates(manip));
        dB_mod((obj.input_num-1)*nq + (1:nq),1:nq) = obj.scale_factor*(J'*daxis_world + reshape(dJ'*axis_world,nq,nq));
        
      else
        kinsol = doKinematics(manip,q);
        [x,J] = forwardKin(manip,kinsol,obj.kinframe,zeros(3,1));
        axis_world = forwardKin(manip,kinsol,obj.kinframe,obj.axis);
        axis_world = axis_world-x;
      end
      
       % find distance to surface and normal at surface
      %Bullet way
         active_collision_options.body_idx = [obj.parent_link;1];
      [phi,surf_norm,xA,xB,idxA,idxB] = manip.collisionDetect(kinsol, ...
                                           false, ...
                                           active_collision_options);
      
      %find directions for forces
      dir_x = cross(axis_world, surf_norm);
      
      dir_y = cross(dir_x, surf_norm);
      if norm(dir_x) == 0 
          warning('spin axis perpendicular to surface');
          v_plate = zeros(3,1);
      else
        y_plate = surf_norm;
        x_plate = cross(axis_world, surf_norm);
        x_plate = x_plate/norm(x_plate);
        z_plate = cross(x_plate,y_plate);
        basis_plate = [x_plate,y_plate,z_plate];
        
        qd_world = J*qd;
        v_world = qd_world(1:3);
        v_plate = basis_plate'*v_world;
        
      end
      
      %convert v_world to v in the surface of the plane
      
     
       %Analytical way
       %TODO
       
       %Find f1 and f2 in the plate frame
       f1 = solveFx(obj,phi,v_plate);
       f2 = solveFy(obj,phi,v_plate);
      
      f_coupler = f1*dir_x + f2*dir_y;
      % apply force along the z-axis of the reference frame
      B_mod(:,obj.input_num) = J'*f_coupler;
      %find directions for forces At low speeds, forces tangential to
      %surface
      dir_x = cross(axis_world, surf_norm); 
      B_mod(:,obj.input_num) = obj.scale_factor*J'*force_dir;
    end
    
            %% SET PROPERTIES
        function obj = set(obj,names, vals)
            if length(names) ~= length(vals)
                error('Names and vals must be the same length')
            end
            for i = 1:length(names)
                if isfield(obj,names{i})
                    obj.(matlab.lang.makevalidname(names{i})) = vals(i);
                else
                    error('Not a field');
                end
            end
            obj.C = findC(obj);
        end
               %% FIND THE FORCE FROM THE COUPLER
        function f = findForce(obj,g,xdot,w_e)
            %fourierForce finds the steady state force from a single em source with a
            %single time frequency.
            
            
            %OUTPUT
            % f- force 2x1 fx,fz
            if strcmp(obj.type,'pm')
               
                B_s = @(xi)obj.halbachSource(xi,obj.C,obj.P,obj.r_o, g,0);
                Gamma = @(xi)obj.findGamma(xi,xdot(1),xdot(2),obj.mu_0, obj.sigma, w_e,obj.b);
                endPt = 1E3; % used to avoid problems at infinity with EM forces
                integrand = @(xi)(abs(B_s(xi)).^2.*Gamma(xi));
                im_force = obj.w/(8*pi*obj.mu_0)*(integral(integrand,-endPt,endPt));
                %negatives to have force on magnet rather than force on plate
                f = [-imag(im_force);-real(im_force)];
            elseif strcmp(obj.type,'em')
                %Need to put em force model here
            end
            
        end
        
        %% FIND THE DERIVATIVE OF THE FORCE WRT W
        function df = findForceDeriv(obj,x,xdot,w_0)
            v_x = xdot(1);
            v_y = xdot(2);
            g = x(2);
            w_e = w_0;
            endPt = 1E3; % used to avoid problems at infinity with EM forces
                
            B_s = @(xi)obj.halbachSource(xi,obj.C,obj.P,obj.r_o, g,0);
            dGamma = @(xi)dGammadW(obj.b,obj.mu_0,obj.sigma,v_x,v_y,w_e,xi);
            integrand = @(xi)(abs(B_s(xi)).^2.*dGamma(xi));
             im_dforce = obj.w/(8*pi*obj.mu_0)*(integral(integrand,-endPt,endPt));
              df = [imag(im_dforce);real(im_dforce)];
        end

        
        %% FIND INTERPOLATION SPLINES
        function [f_x_spline, f_y_spline] = findSplines(obj, divs)
            
            v_x_min = obj.v_lims(1,1);
            v_x_max = obj.v_lims(1,2);
            v_y_min = obj.v_lims(2,1);
            v_y_max = obj.v_lims(2,2);
            g_min = obj.g_lims(1);
            g_max = obj.g_lims(1);
            w_min = obj.w_lims(1);
            w_max = obj.w_lims(2);
            
            v_x = [v_x_min:(v_x_max-v_x_min)/divs:v_x_max];
            v_y = [v_y_min:(v_y_max-v_y_min)/divs:v_y_max];
            g = [g_min:(g_max-g_min)/divs:g_max];
            w_e = [w_min:(w_max-w_min)/divs:w_max];
            
            
            Fx = zeros(length(v_x), length(v_y), length(g), length(w));
            Fy = zeros(length(v_x), length(v_y), length(g), length(w));
            disp('Finding forces over workspace ...');
            for i = 1:length(v_x)
                for j = 1:length(v_y)
                    for k = 1:length(g)
                        for l = 1:length(w_e)
                            F = c.findForce([0,g(k)],[v_x(i) v_x(j)],w(l));
                            Fx(i,j,k,l) = F(1);
                            Fy(i,j,k,l) = F(2);
                        end
                    end
                end
            end
            disp('Solving for splines')
            f_x_spline = csapi( {v_x,v_y,g,w}, Fx );
            f_y_spline = csapi( {v_x,v_y,g,w}, Fy );
        end
                     %% FIND CONSTANT FOR HALBACH MAGNET
        function C = findC(obj)
            %find constant C for a halbach rotor
            %variables
            %Br - magnet remittance
            %P
            %ur - relative permeability
            %r_o - outer radius
            % r_i - inner radius
            
            C = -(2*obj.Br*obj.P)/(obj.P+1)*((1+obj.ur)*obj.r_o^(2*obj.P)*(obj.r_o^(obj.P+1)-obj.r_i^(obj.P+1)))/...
                ((1-obj.ur)^2*obj.r_i^(2*obj.P)-(1+obj.ur)^2*obj.r_o^(2*obj.P));
        end
    
  end
  
  methods (Static)
   function [model,obj] =  parseURDFNode(model,robotnum,node,options)
       %standard urdf 
      name = char(node.getAttribute('name'));
      name = regexprep(name, '\.', '_', 'preservecase');
      
      elnode = node.getElementsByTagName('parent').item(0);
      parent = findLinkInd(model,char(elnode.getAttribute('link')),robotnum);
      
      
      xyz = zeros(3,1); rpy = zeros(3,1);
      elnode = node.getElementsByTagName('origin').item(0);
      if ~isempty(elnode)
        if elnode.hasAttribute('xyz')
          xyz = reshape(parseParamString(model,robotnum,char(elnode.getAttribute('xyz'))),3,1);
        end
        if elnode.hasAttribute('rpy')
          rpy = reshape(parseParamString(model,robotnum,char(elnode.getAttribute('rpy'))),3,1);
        end
      end
      
      fr = RigidBodyFrame(parent,xyz,rpy,[name,'_frame']);
      [model,frame_id] = addFrame(model,fr);
      
      scaleFac = 1;
      if node.hasAttribute('scale_factor')
        scaleFac = parseParamString(model,robotnum,char(node.getAttribute('scale_factor')));
      end
      
      axis = [1; 0; 0];
      elnode = node.getElementsByTagName('axis').item(0);
      if ~isempty(elnode)
        axis = reshape(parseParamString(model,robotnum,char(elnode.getAttribute('xyz'))),3,1);
        axis = axis/(norm(axis)+eps); % normalize
      end
      
      limits = [-inf,inf];
      if node.hasAttribute('lower_limit')
        limits(1) = parseParamString(model,robotnum,char(node.getAttribute('lower_limit')));
      end
      if node.hasAttribute('upper_limit')
        limits(2) = parseParamString(model,robotnum,char(node.getAttribute('upper_limit')));
      end
      
  obj = RigidBodyCoupler(frame_id, axis, scaleFac, limits);
      obj.name = name;
      obj.parent_link = parent; %remember parent link for collision checking
   end
   
   function B_s = halbachSource(xi, C, P, r_o, g, x)
            
            B_s = 4*pi*C*xi.^P/factorial(P).*exp(-xi.*(r_o + g +1i*x)).*(xi >= 0);
            B_s(isnan(B_s))= 0;
           end
        
            function Gamma = findGamma(xi, v_x, v_y, mu_0, sigma, w_e,b)
            %finds the big gamma transmission propigation function from the bird style
            %steady-state force solution to eddy current propigation
            %INPUTS
            %OUTPUT
            
            s0 = 1i*(w_e*ones(size(xi)) - xi*v_x);
            lambda = v_y * mu_0*sigma/2*ones(size(xi));
            gamma2 = xi.^2+mu_0*sigma*s0.*ones(size(xi));
            beta2 = lambda.^2+gamma2;
            beta = sqrt(beta2);
            
            Tss_top = ((lambda-(xi+beta)).*exp(beta*b)-(lambda - (xi - beta)).*exp(-beta*b));
            Tss_bottom = exp(beta*b).*(lambda.^2-(xi + beta).^2)-exp(-beta*b).*(lambda.^2-(xi-beta).^2);
            Tss = Tss_top./Tss_bottom;
            
            Gamma = 2.*xi.*Tss - 1;
            
        end

  end
 
properties
      kinframe %id number of the object's frame
      axis % the coupler's axis in it's frame
      scale_factor = 1; %amount to scale the input by.
      parent_link = 0; %parent link number of the coupler - relevant for finding distances
      use_bullet;
      C; %meta magnet property
      %MAGNET PROPERTIES
      m = 2E-3; %Am^2
      r_o = 2.54*0.75*0.01; %0.75 inches converted to m
      r_i = 2.54*0.5*0.01;  %0.5 inches converted to m
      Br = 1.42; %T Magnet strength
      ur = 1.08; %unitless magnet relative permeability
      P = 1; %pole-pairs
      w = 0.01; %width of the magnet
      mu_0 = 1.256E-6; %m*kg/s^2 A^2 magnetic permeability in a vacuum
      type = 'pm';%indicate whether an electromagnet or permanent magnet
      %PLATE PROPERTIES
      rho = 2.83E-6; %ohm-m resistivity of the plate
      sigma; %conductivity of the plate
      
      b = 0.005; %plate thickness
      %STATE PROPERTIES
      x_lims = [-10 10; 0.001 2; -pi pi];
      v_lims = [-10 10; -10 10]; %x velocity parallel to the plate
      %y velocity perpendicular to the plate
      g_lims = [0.001 2]; %air gap m
      w_e_lims = [-10000 10000]; %electric frequency
  end
  
end
