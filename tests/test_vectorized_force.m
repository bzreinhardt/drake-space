%Test Findforce
%load('force_torque_structs.mat');
%generate new splines
divs = struct();
divs.g = 15;
divs.v_x = 15;
divs.v_y = 15;
divs.w = 5;
% c = InductionCouplerModel();
%  c = c.setSplines( divs);
%          f_x_spline = c.f_x; f_y_spline = c.f_y;
%          save('force_torque_structs_detailed.mat','f_x_spline','f_y_spline');
w = [10;20];
a = [0 1 0; 0 1 0];
g = [0.2;0.1];
v = zeros(2,3);
surf_norm = [0 0 1; 0 0 1];
%% test splines
u = linspace(-10000,10000,100);
g_test = ones(size(u))*0.01;
vx = zeros(size(u));
vy = zeros(size(u));
f_x = fnval(f_x_spline,[vx;vy;g_test;u]);
f_y = fnval(f_y_spline,[vx;vy;g_test;u]);

figure(1337); subplot(211);plot(u,f_x);
subplot(212);plot(u,f_y);

[F] = findForce(w, a, g, v, surf_norm, f_x_spline, f_y_spline);

X = [0;0;10.1;zeros(9,1)];

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

[x,v,a_world] = testKinematics(X,a,d)

r = 10; c = zeros(3,1);
[g, surf_norm] = sphereNormGap(x, r, c)
