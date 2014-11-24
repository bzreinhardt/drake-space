function phaseout = planarDynamics(input)
%@param coupler_model_structs are the structs with the planar behavior of
%the induction couplers

%note - a axes and d vectors are row vectors

%All inputs are MxN vectors with M time steps and N channels

auxdata = input.auxdata;
SPHERE_RADIUS = auxdata.r;
SPHERE_CENTER = auxdata.c;
a = auxdata.a;
d = auxdata.d;

x     = input.phase.state(:,1);
y     = input.phase.state(:,2);
theta = input.phase.state(:,3);
vx    = input.phase.state(:,4);
vy    = input.phase.state(:,5);
omega = input.phase.state(:,6);
u     = input.phase.control;
u1    = input.phase.control(:,1);
u2    = input.phase.control(:,2);



% TODO add in velocity dynamics - ignore for now
vx_plate = zeros(size(vx));
vy_plate = zeros(size(vy));


net_force_x = zeros(size(vx));
net_force_y = zeros(size(vy));
net_torque = zeros(size(omega));
%cycle through the forces and torques from each coupler

for i = 1:size(a,1)
    %find coupler positions in world coordinates
    coupler_x = d(i,1)*cos(theta)-d(i,2)*sin(theta)+x;
    coupler_y = d(i,1)*sin(theta)+d(i,2)*cos(theta)+y;
    
    [g,surf_norm] = sphereNormGap([coupler_x,coupler_y],SPHERE_RADIUS,SPHERE_CENTER);
    
    if any(any(isnan([g,vx_plate,vy_plate,u(:,i)])))
        fx = NaN*net_force_x;
        fy = NaN*net_force_y;
    else
    fx = fnval(input.auxdata.fx,[vx_plate,vy_plate,g,u(:,i)]')';
    fy = fnval(input.auxdata.fy,[vx_plate,vy_plate,g,u(:,i)]')';
    end
   
    force_x = -surf_norm(:,2).*fx + surf_norm(:,1).*fy;
    force_y = surf_norm(:,1).*fx + surf_norm(:,2).*fy;
    torque = -(d(i,1)*sin(theta)+d(i,2)*cos(theta)).*force_x + ...
        (d(i,1)*cos(theta)-d(i,2)*sin(theta)).*force_y;
    
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

phaseout.dynamics = [xdot, ydot, thetadot, vxdot, vydot, omegadot];
%phaseout.path = (x-SPHERE_CENTER(1)).^2 + (y - SPHERE_CENTER(1)).^2; 
phaseout.path = (x-SPHERE_CENTER(1)).^2+(y-SPHERE_CENTER(2)).^2;
phaseout.integrand = 0.001*sqrt(u1.^2 + u2.^2);

%