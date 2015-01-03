function dX = couplerDynamics(t,X,u,options)
%COUPLERDYNAMICS finds the forces and torques from a single induction
%coupler
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

%defaults 
if nargin < 4
    options.planner = 'drake';
end
    
    a = [0;0;1]; % assume axis points out of the page unless told otherwise
    radius = 5;
    center = [0;-5;0];
    
    function [g,n] = findNormGap
    g = sqrt(sum([x;y;z] - center).^2,2) - radius;
    n = ([x;y;z] - center)./(g + radius);
    end
    %deal with options

    if strcmp(options.planner,'drake')
        if size(x,1) == 6
        elseif size(x,1) == 7
        end
        
    elseif strcmp(options.planner, 'gpops')
    else
    end

%planar drake dynamics
x = X(1);
y = X(2);
theta = X(3);
vx = X(4);
vy = X(5);
omega = X(6);
z = 0;
vz = 0;
[g,n] = findNormGap;


%force in surface frame
fx = (-5.6219e-08.*u.^3+6.2003e-06.*u.^2+76.0622.*u)...
    .*6.2003e-06.*exp(-44.1463.*g);

fy = (5.811e-09.*u.^4+-2.2676e-08.*u.^3+-1.8637...
    .*u.^2+0.29795.*u).*-33.8115.*exp(-38.9901.*g+-21.8594);

%force in global frame
%F = Fx(g,w,v)(axn) + Fy(axn)xa
axn = cross(a,n,1);
axnxa = cross(axn,a,1);
dX = fx*axn + fy*axnxa;



end

