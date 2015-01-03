function genPlanarCouplerGradient(filename)
%Finds the system gradients for a single induction coupler
% operating in the plane
% assumes axis is in +z direction
%todo, update to create function interface that is more useful 
t = sym('t');
    center = [sym('c1');sym('c2')];
    radius = sym('r');
    d1 = sym('d1');
    d2 = sym('d2');
    x = sym('x');
    y = sym('y');
    theta = sym('theta');
    vx = sym('vx');
    vy = sym('vy');
    omega = sym('omega');
    u = sym('u');
    state = [x;y;theta;vx;vy;omega];
        
    d_w = sym(zeros(2,1));
    %find coupler positions in world coordinates
    d_w(1) = d1*cos(theta)-d2*sin(theta)+x;
    d_w(2) = d1*sin(theta)+d2*cos(theta)+y;
    
    g = sqrt(sum((d_w - center).^2))-radius;
    
    surf_norm = (d_w - center)./...
        sqrt(sum((d_w - center).^2));
    
    fx = (-5.6219e-08.*u.^3+6.2003e-06.*u.^2+76.0622.*u)...
                    .*6.2003e-06.*exp(-44.1463.*g);
                
                fy = (5.811e-09.*u.^4+-2.2676e-08.*u.^3+-1.8637...
                    .*u.^2+0.29795.*u).*-33.8115.*exp(-38.9901.*g+-21.8594);
    
   
    force_x = -surf_norm(2).*fx + surf_norm(1).*fy;
    force_y = surf_norm(1).*fx + surf_norm(2).*fy;
    
    torque = -d_w(2).*force_x + d_w(1).*force_y;

    %xdot = couplerDynamics(t,state,u,d,[0;0;1])
xdot = [sym(zeros(3,1));...
    force_x;...
    force_y;...
    torque];

dxdot = [jacobian(xdot,t),jacobian(xdot,state),jacobian(xdot,u)];
matlabFunction(dxdot,'file',filename);
