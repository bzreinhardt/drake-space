%Test the optimization of forces and torques for a three armed guy
d1 = [1/2^.5,1/2^.5,-0.1];
d2 = [-1/2^.5,1/2^.5,-0.1];
d3 = [-1/2^.5,-1/2^.5,-0.1];
d4 = [1/2^.5,-1/2^.5,-0.1];

d = [d1;d2;d3;d4];
a1 = d1/norm(d1);
a2 = d1/norm(d2);
a3 = d1/norm(d3);
a4 = d1/norm(d4);

a = [a1;a2;a3;a4];

%create an inductorcouplermodel to find the splines

c = InductionCouplerModel();
divs = struct();
divs.g = 15;
divs.v_x = 15;
divs.v_y = 15;
divs.w = 5;
if isempty(c.f_x)
c = c.setSplines( divs);
end
f_x_spline = c.f_x;
f_y_spline = c.f_y;
%do a few tests to make sure force/torque model is working
ft_net = zeros(6,1);
u = [10 10 10 10];
r = 10; % 10 m sphere
center = zeros(3,1);
X = [0;0;10.07;zeros(9,1)];
for i = 1:length(u)
    ft = findForceTorque(X,d(i,:),u(i),a(i,:),r,center,f_x_spline,f_y_spline)
    ft_net = ft_net+ft;
end


