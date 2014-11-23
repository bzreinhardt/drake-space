% TEST_extensible_inverseActuator

n = 25;
num_u = 2;
u_max = 1E4;
u_min = -u_max;
%grid of possible inputs
u1 = linspace(u_min,u_max,n);
u2 = linspace(u_min,u_max,n);

%Generate a target force
xTarget = rand(1);
yTarget = rand(1);
zTarget = -0.05 + 0.1*rand(1);
fTarget = [xTarget;yTarget;zTarget];  %Target Force

%f1 = actuator(u1);
%f2 = actuator(u2);
%Set up the state
X = [0;0;10.1;zeros(9,1)];

d1 = [1/2^.5,1/2^.5,-0.1];
d2 = [-1/2^.5,1/2^.5,-0.1];

d = [d1;d2];
a1 = d1/norm(d1);
a2 = d2/norm(d2);

a = [a1;a2];

[x,v,a_world] = testKinematics(X,a,d);

r = 10; c = zeros(3,1);
[g, surf_norm] = sphereNormGap(x, r, c);

[U1, U2] = meshgrid(u1,u1);
% u is a 2xn*n vector of all the possibible (discretized) input combinations
% I suspect this ceases to work 
u = [reshape(U1,1,n*n); reshape(U2,1,n*n)];
actuator = @(u)findNetForce(u,a_world,g,v,surf_norm,f_x_spline,f_y_spline);
cost = objective(u,fTarget,actuator);
[~,idx] = min(cost,[],2);

%closest u in u grid
uGuess = u(:,idx);
% foce generated by closest u
fGuess = actuator(uGuess);
f_prev = [];
u_soln = zeros(size(uGuess));
f_soln = zeros(size(fGuess));

    Problem.x0 = uGuess;
    Problem.objective = @(u)objective(u,fTarget,actuator);
    Problem.Aeq = [];
    Problem.beq = [];
    Problem.Aineq = [];
    Problem.bineq = [];
    %limits on u
    Problem.lb = u_min*ones(num_u,1);
    Problem.ub= u_max*ones(num_u,1);
    Problem.options = optimset();
    Problem.solver = 'fmincon';
    % minimize difference between the generated force and the actual force
    u_soln(:) = fmincon(Problem);
f_soln(:) = actuator(u_soln);


figure(1338); clf; hold on;
plot3(fTarget(1),fTarget(2), fTarget(3),'rx','MarkerSize',15,'LineWidth',4)
plot3(fGuess(1),fGuess(2), fGuess(3),'go','MarkerSize',20,'LineWidth',4)
plot3(f_soln(1),f_soln(2), f_soln(3),'bx','MarkerSize',15,'LineWidth',4)
plotActuator(actuator);
xlabel('x');
ylabel('y');
zlabel('z');
legend('Target','Guess','Soln');