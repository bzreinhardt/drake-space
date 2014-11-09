function test_control_optimization
%Test the optimization of forces and torques for a three armed guy
d1 = [1/2^.5,1/2^.5,-0.1];
d2 = [-1/2^.5,1/2^.5,-0.1];
d3 = [-1/2^.5,-1/2^.5,-0.1];
d4 = [1/2^.5,-1/2^.5,-0.1];

d = [d1;d2;d3;d4];
a1 = d1/norm(d1);
a2 = d2/norm(d2);
a3 = d3/norm(d3);
a4 = d4/norm(d4);

a = [a1;a2;a3;a4];

%create an inductorcouplermodel to find the splines


divs = struct();
divs.g = 15;
divs.v_x = 15;
divs.v_y = 15;
divs.w = 5;
if exist('c','var')
if ~isa(c,'InductionCouplerModel')
    c = InductionCouplerModel();
end
else
    c = InductionCouplerModel();
end
if ~exist('f_x_spline','var')
     if exist('force_torque_structs.mat','file')
         load('force_torque_structs.mat')
         c.f_x = f_x_spline; c.f_y = f_y_spline;
     else
         c = c.setSplines( divs);
         f_x_spline = c.f_x; f_y_spline = c.f_y;
         save('force_torque_structs.mat','f_x_spline','f_y_spline');
     end
 end

 

%do a few tests to make sure force/torque model is working
f_net = zeros(3,1);
t_net = zeros(3,1);
u0 = [10 10 10 10];
r = 10; % 10 m sphere
center = zeros(3,1);
X = [0;0;10.1;zeros(9,1)];
for i = 1:length(u0)
    w_R_b = rpy2rotmat(X(4:6));
    a_world = w_R_b*a(i,:)';
    d_world = w_R_b*d(i,:)';
    x = X + [w_R_b*d(i,:)';zeros(9,1)];
    
    f = findForceTorque(x,u0(i),a_world,r,center,[f_x_spline],[f_y_spline])
    t = cross(d_world,f);
    f_net = f_net + f;
    t_net = t_net + t;
end
    function cost = objective(u,f_i)
        %maximize component of force f_i
        f_net = zeros(3,1);
        for i = 1:length(u)
            a_world = w_R_b*a(i,:)';
            d_world = w_R_b*d(i,:)';
            x = X + [w_R_b*d(i,:)';zeros(9,1)];
            
            f = findForceTorque(x,u(i),a_world,r,center,[f_x_spline],[f_y_spline]);
            f_net = f_net + f;
           
        end
        cost = 1/f_net(f_i);
    end
LB = -10000;
UB = 10000;
outputs = zeros(3,length(u0));
for i = 1:3
    outputs(i,:) = fmincon(@(u)objective(u,i),zeros(1,4),[],[],[],[],LB*ones(1,4),UB*ones(1,4));
end

end


