%TEST_numInvActuator

X = [0;0;10.1;zeros(9,1)];

d1 = [1/2^.5,1/2^.5,-0.1];
d2 = [-1/2^.5,1/2^.5,-0.1];
d3 = [1/2^.5,-1/2^.5,-0.1];

%Generate a target force
xTarget = rand(1);
yTarget = rand(1);
zTarget = -0.05 + 0.1*rand(1);
f_target = [xTarget;yTarget;zTarget];  %Target Force


a1 = d1/norm(d1);
a2 = d2/norm(d2);
a3 = d3/norm(d3);

a = [a1;a2;a3];
d = [d1;d2;d3];

[x,v,a_world] = testKinematics(X,a,d);

r = 10; c = zeros(3,1);
[g, surf_norm] = sphereNormGap(x, r, c);
%actual physics of the actuator
actuator = @(u)findNetForce(u,a_world,g,v,surf_norm,f_x_spline,f_y_spline);

[u_soln, f_soln, f_guess] = numInvActuator(3,f_target,actuator);

figure(1338); clf; hold on;
plot3(f_target(1),f_target(2), f_target(3),'rx','MarkerSize',15,'LineWidth',4)
plot3(f_guess(1),f_guess(2), f_guess(3),'go','MarkerSize',20,'LineWidth',4)
plot3(f_soln(1),f_soln(2), f_soln(3),'bx','MarkerSize',15,'LineWidth',4)
plotActuator(actuator,3);
xlabel('x');
ylabel('y');
zlabel('z');
legend('Target','Guess','Soln');