function [out1, out2] = drakeifiedPlanarDynamics(varargin)
%@param coupler_model_structs are the structs with the planar behavior of
%the induction couplers

%note - a axes and d vectors are row vectors

%All inputs are MxN vectors with M time steps and N channels

if nargin < 2
    input = varargin{1};
auxdata = input.auxdata;
radius = auxdata.r;
center = auxdata.c;
a = auxdata.a;
d = auxdata.d;

x     = input.phase.state(:,1);
y     = input.phase.state(:,2);
theta = input.phase.state(:,3);
vx    = input.phase.state(:,4);
vy    = input.phase.state(:,5);
omega = input.phase.state(:,6);
u     = input.phase.control;

else
    obj = varargin{1};
    t = varargin{2};
    X = varargin{3};
    u = varargin{4};
    if size(u,1) == obj.getNumInputs
        u = u';
    end
    d = obj.d;
    center = obj.sphere_center;
    radius = obj.sphere_radius;
    x = X(1);
    y = X(2);
    theta = X(3);
    vx = X(4);
    vy = X(5);
    omega = X(6);
end
net_force_x = zeros(size(vx));
net_force_y = zeros(size(vy));
net_torque = zeros(size(omega));
d_w = zeros(size(theta,1),2);
for i = 1:size(d,1)
    %find coupler positions in world coordinates
    d_w(:,1) = d(i,1)*cos(theta)-d(i,2)*sin(theta)+x;
    d_w(:,2) = d(i,1)*sin(theta)+d(i,2)*cos(theta)+y;
    
    g = sqrt(sum((d_w - ones(size(d_w,1),1)*center').^2,2))-radius;
    
    surf_norm = (d_w - ones(size(d_w,1),1)*center')./...
        (sqrt(sum((d_w - ones(size(d_w,1),1)*center').^2,2))*ones(1,size(d_w,2)));
    
    fx = (-5.6219e-08.*u(:,i).^3+6.2003e-06.*u(:,i).^2+76.0622.*u(:,i))...
                    .*6.2003e-06.*exp(-44.1463.*g);
                fy = (5.811e-09.*u(:,i).^4+-2.2676e-08.*u(:,i).^3+-1.8637...
                    .*u(:,i).^2+0.29795.*u(:,i)).*-33.8115.*exp(-38.9901.*g+-21.8594);
    
   
    force_x = -surf_norm(:,2).*fx + surf_norm(:,1).*fy;
    force_y = surf_norm(:,1).*fx + surf_norm(:,2).*fy;
    torque = -d_w(:,2).*force_x + ...
        d_w(:,1).*force_y;
    
    net_force_x = net_force_x + force_x;
    net_force_y = net_force_y + force_y;
    net_torque = net_torque + torque;
end

xdot = vx;
ydot = vy;
thetadot = omega;
vxdot = net_force_x;
vydot = net_force_y;
omegadot = net_torque;

if nargin == 1 
    
phaseout.dynamics = [xdot, ydot, thetadot, vxdot, vydot, omegadot];
%phaseout.path = (x-SPHERE_CENTER(1)).^2 + (y - SPHERE_CENTER(1)).^2; 
phaseout.path = (x-center(1)).^2+(y-center(2)).^2;
%phaseout.integrand = 0.001*sqrt(u1.^2 + u2.^2)+0.001*(u1.^2.+u2.^2).*input.phase.time;
phaseout.integrand = 0.001*(sum(u.^2,2))+sum(vx.^2)+sum(vy.^2);
out1 = phaseout;
else
    out1 = [xdot, ydot, thetadot, vxdot, vydot, omegadot]';
    if nargout > 1
    %out2 = planarInspectorGradients(obj,t,X,u',1);
    out2 = ones(obj.getNumStates,1+obj.getNumStates+obj.getNumInputs);
    end
end



end
