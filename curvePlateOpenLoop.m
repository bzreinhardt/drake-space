function curvePlateOpenLoop
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    r = InductionInspector('curve_inspector_4_couplers.urdf');
     
     r = r.addCurvedSurface(7,2, [0;0;-7],[0;pi/2;0]);
      sys = TimeSteppingRigidBodyManipulator(r,.01);
      
     v = sys.constructVisualizer();

      x0 = [0;0;0.4;zeros(17,1)];
      u0 = Point(getInputFrame(r),[-0.008; 0.008;0;0]);
      
      sys = cascade(ConstantTrajectory(u0),sys);

%      sys = cascade(sys,v);
%      simulate(sys,[0 2],double(x0)+.1*randn(12,1));
      
      options.capture_lcm_channels = 'LCMGL';
      [ytraj,xtraj,lcmlog] = simulate(sys,[0 5],double(x0),options);
      
   %   [ytraj2,xtraj2] = simulateODE(sys,[0,2],double(x0),options)
      lcmlog
      %v.playback(xtraj,struct('lcmlog',lcmlog));
      figure(1); clf; fnplt(ytraj,3);
      v.playback(xtraj);

end

