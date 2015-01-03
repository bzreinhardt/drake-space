function genPlanarCouplerGradientFromFcn(filename)
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
    X = [x;y;theta;vx;vy;omega];

    options.radius = radius;
    options.center = center;
    options.d = [d1;d2];
    xdot = couplerDynamics2D(t,X,u,options);
    dxdot = [jacobian(xdot,t),jacobian(xdot,X),jacobian(xdot,u)];
matlabFunction(dxdot,'file',filename);

%put the file in useful form 
fid = fopen(filename);

call = fgetl(fid); %get first line
fclose(fid);
args = getargs(call);

    