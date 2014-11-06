function xtraj = runLQR

r = InductionInspector('models/curve_inspector_6_couplers.urdf');
 r = r.addCurvedSurface(7,2, [0;0;-7],[0;pi/2;0]);
v = r.constructVisualizer();

x_nom = [0;0;0.25;zeros(21,1)];
u0_point = Point(getInputFrame(r),[0; 0;0;0;0;0]);
u0 = double(u0_point);

%c = tilqr(r,x0,u0,diag([10*ones(5,1);0;ones(5,1);0]),eye(4));

% the linearized system 
[A,B] = linearize(r,0,x_nom,double(u0));

Q = diag([10*ones(12,1); ones(12,1)]);
R = .1*eye(6);
K = lqr(full(A),full(B),Q,R);

% u = u0 - K*(x-x0)
c = AffineSystem([],[],[],[],[],[],[],-K,u0 + K*x0);
c = setInputFrame(c,getStateFrame(r));
c = setOutputFrame(c,getInputFrame(r));

sys = feedback(r,c);

%for i=1:5
x0 = [0;0;0.496;zeros(21,1)];
xtraj = simulate(sys,[0 4],double(x0));
figure(1);clf;fnplt(xtraj,3);
  %xtraj = simulate(sys,[0 4],double(x0)+[.5*randn(6,1);zeros(6,1)]);
  v.playback(xtraj);
%end