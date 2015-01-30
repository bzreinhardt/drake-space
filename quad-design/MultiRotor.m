classdef MultiRotor < SecondOrderSystem

  % state:  
  %  q(1) - x position
  %  q(2) - z position
  %  q(3) - pitch (theta)
  % input:
  %  u(1) - prop 1 thrust
  %  u(2) - prop 2 thrust

  properties  %  based on (Bouadi, Bouchoucha, Tadjine 2007)
    l = [-0.25; 0.25]; % lengths of rotor arms
    m = 0.486; % mass of quadrotor
    I = 0.00383; % moment of inertia
    g = 9.81; % gravity
    angles = [pi/2; pi/2];
    
  end
  
  methods
    function obj = MultiRotor(angles, l)
        if nargin < 1
            num_rotors = 2;
        else
            num_rotors = numel(angles);
        end
        
      obj = obj@SecondOrderSystem(3,num_rotors,true);
      obj = obj.setOutputFrame(obj.getStateFrame);  % allow full-state feedback
      
      if nargin > 0
          obj.angles = angles;
      end
      if nargin > 1
          obj.l = l;
      end
      
      obj.m = 0.4248+0.0306*numel(obj.angles);
      obj.I = sum(0.0306*obj.l.^2);
      %u_max = 10;
      obj = setInputLimits(obj,0,10);
      

      
      if numel(obj.angles) ~= numel(obj.l)
          error('MULTIROTOR: number of rotors does not match number of rotor positions. numel(angles) needs to match numel(l)');
      end
      
      
    end
    
    function qdd = sodynamics(obj,t,q,qd,u)
      % Implement the second-order dynamics
      u_x = sum(cos(obj.angles).*u);
      u_y = sum(sin(obj.angles).*u);
      u_tau = sum(obj.l.*sin(obj.angles).*u);
      
      qdd = [ 1/obj.m *( cos(q(3))*u_x - sin(q(3))*u_y);
      1/obj.m*(sin(q(3))*u_x + cos(q(3))*u_y) - obj.g;
        1/obj.I*(u_tau)];
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
    
  end
  
end
