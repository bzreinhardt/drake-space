classdef DoublePendPlant < Manipulator
  
  properties
    % parameters from Spong95 (except inertias are now relative to the
    % joint axes)
    l1 = 1; l2 = 1;  
    m1 = 1; m2 = 1;  
    g = 9.81;
    b1=0;  b2=0;
    I1=[]; I2=[]; % set in constructor
    u_max = 5;
    u_min = -5;
  end
  
  methods
      
      function obj = DoublePendPlant(l,m,b,I)
          obj = obj@Manipulator(2,2);
          if nargin > 0
              obj.l1 = l(1);  obj.l2 = l(2);
          end
          if nargin > 1
              obj.m1 = m(1);  obj.m2 = m(2);
          end
          if nargin > 2
              obj.b1 = b(1);  obj.b2 = b(2);
          end
          if nargin > 3
              obj.I1 = I(1); obj.I2 = I(2);
          else
              obj.I1 = obj.m1*obj.l1^2;
              obj.I2 = obj.m2*obj.l2^2;
          end
          obj = obj.setOutputFrame(obj.getStateFrame);
          obj = obj.setInputLimits(obj.u_min, obj.u_max);
    end
    
    function [H,C,B] = manipulatorDynamics(obj,q,qd)
      % keep it readable:
      m1=obj.m1; m2=obj.m2; l1=obj.l1; l2=obj.l2; g=obj.g; b1=obj.b1; b2=obj.b2; I1=obj.I1; I2=obj.I2;
      m2l1l2 = m2*l1*l2;  % occurs often!

      c = cos(q(1:2,:));  s = sin(q(1:2,:));  s12 = sin(q(1,:)+q(2,:));
      
      h12 = I2 + m2l1l2*c(2);
      H = [ I1 + I2 + m2*l1^2 + 2*m2l1l2*c(2), h12; h12, I2 ];
      
      C = [ -2*m2l1l2*s(2)*qd(2), -m2l1l2*s(2)*qd(2); m2l1l2*s(2)*qd(1), 0 ];
      G = g*[ m1*l1*s(1) + m2*(l1*s(1)+l2*s12); m2*l2*s12 ];
            
      % accumate total C and add a damping term:
      C = C*qd + G + [b1;b2].*qd;

      B = eye(2);
    end
    
    function tau = equilibriumTorque(obj,q,warn)
        if nargin < 3
            warn = false;
        end
        [~,C,B] = obj.manipulatorDynamics(q,zeros(obj.getNumStates/2,1));
        if ~all(C < B*obj.umax)
            if warn
            warning('DOUBLEPENDULUM EQUILIBRIUMTORQUE: equilibrium torque exceeds max');
            end
            tau = [NaN;NaN];
        elseif ~all(C > B*obj.umin)
            if warn
            warning('DOUBLEPENDULUM EQUILIBRIUMTORQUE: equilibrium torque under min');
            end
            tau = [NaN;NaN];
        else
            tau = C;
        end
    end
    
    function c = genControlRegion(obj,x0)
        c = PendulumControlRegion(x0,obj);
    end
    
    function x = getInitialState(obj)
      x = .1*randn(4,1);
    end
    
     function data = write(obj)
             %output a struct representing the object
             data.m1 = obj.m1;
              data.m2 = obj.m2;
             data.l1 = obj.l1;
             data.l2 = obj.l2;
         end
    
  end
  
  methods(Static)
    function run()  % runs the passive system
      pd = DoublePendPlant;
      pv = DoublePendVisualizer(pd);
      traj = simulate(pd,[0 5],randn(4,1));
      playback(pv,traj);
    end
  end
  
end
