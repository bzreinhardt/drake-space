function generateInspectorGradients(p,filename)
%todo, update to create function interface that is more useful 
t = sym('t');
    d = p.d;
    center = p.sphere_center;
    radius = p.sphere_radius;
    x = sym('x');
    y = sym('y');
    theta = sym('theta');
    vx = sym('vx');
    vy = sym('vy');
    omega = sym('omega');
    u = sym(zeros(1,p.getNumInputs));
    for i = 1:p.getNumInputs
        u(i) = sym(strcat('u',num2str(i)));
    end
    state = [x;y;theta;vx;vy;omega];
        
net_force_x = sym(zeros(size(vx)));
net_force_y = sym(zeros(size(vy)));
net_torque = sym(zeros(size(omega)));
d_w = sym(zeros(size(theta,1),2));
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

xdot = sym([zeros(3) eye(3);zeros(3,6)])*state + [sym(zeros(3,1));net_force_x;net_force_y;net_torque];

dxdot = [jacobian(xdot,t),jacobian(xdot,state),jacobian(xdot,u)];
matlabFunction(dxdot,'file',filename);


