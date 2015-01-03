function dX = couplerDynamics2D(t,X,u,options)
%COUPLERDYNAMICS finds the forces and torques from a single induction
%coupler
%@param obj - parent object
%@param t - time (shouldn't matter)
%@param X - state of the coupler in the global frame 
%@param u - frequency of the coupler
%@options:
%@surface - sphere.radius and sphere.center if pre-specifying the surface
%           equation if specifying by an equation
%           bullet if using contact information from bullet collision det.
%           @default: a sphere radius 5m centered on 0,-5,0
%@planner - drake if using drake's collocation optimizer
%           gpops if using gpops for optimization
%           @default: TODO
%@physics - three different levels of physics possible
%             full - run full simulation each time step (slowest, most truthy)
%             spline - find forces from precomputed splines (slow, less truthy)
%             fit - find forces from a polynomial fit (fastest, least truthy)

%default values
d = [0;-1/sqrt(2);0];
options.planner = 'drake';
radius = 5;
center = [0;-5];

if nargin > 3
    %parse options
    if isfield(options, 'd')
    d = options.d;
    end
    if isfield(options,'radius')
        radius = options.radius;
    end
    if isfield(options,center)
        center = options.center;
    end
end

    function [g,n] = findNormGap
        g = sqrt(sum((d_w - center).^2,1)) - radius;
        n = (d_w - center)./(g + radius);
    end

x = X(1);
y = X(2);
theta = X(3);
vx = X(4);
vy = X(5);
omega = X(6);

xc = cos(theta)*d(1)-sin(theta)*d(2) + x;
yc = sin(theta)*d(1) + cos(theta)*d(2) +y;
d_w = [xc;yc];

%find the gap to the surface
[g,n] = findNormGap;


%force in surface frame
fx = (-5.6219e-08.*u.^3+6.2003e-06.*u.^2+76.0622.*u)...
    .*6.2003e-06.*exp(-44.1463.*g);

fy = (5.811e-09.*u.^4+-2.2676e-08.*u.^3+-1.8637...
    .*u.^2+0.29795.*u).*-33.8115.*exp(-38.9901.*g+-21.8594);

%force in global frame
 force_x = -n(2).*fx + n(1).*fy;
    force_y = n(1).*fx + n(2).*fy;
    torque = -d_w(2).*force_x + d_w(1).*force_y;
    dX = [force_x; force_y; torque];
end

