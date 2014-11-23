
options = [];
options.floating = true;
%m = RigidBodyManipulator('two_coupler_inspector.urdf',options);
 r = InductionInspector('models/two_coupler_inspector.urdf');
r = r.addTargetSurface([5,5,0.01],[0;0;0],[0;0;0]);
% %r.addRobotFromURDF(r,'plate.urdf',zeros(3,1),zeros(3,1));
x0 = [0;0;0.1;0;0;0;0;0];
v0 = zeros(8,1);
v = r.constructVisualizer();

 [force2,B_mod2] = r.force{1,1}.computeNonAffineForce(r,x0,zeros(8,1),300);

 u0 = Point(getInputFrame(r), [-30;30]);
 sys = cascade(ConstantTrajectory(u0),r);
 
 options.capture_lcm_channels = 'LCMGL';
      [ytraj,xtraj,lcmlog] = simulate(sys,[0 10],double([x0;v0]),options);
      v.playback(xtraj,struct('lcmlog',lcmlog));