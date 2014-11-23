clear all;
options = [];
options.floating = true;
%m = RigidBodyManipulator('two_coupler_inspector.urdf',options);
 r = InductionInspector('models/two_coupler_inspector.urdf');
r = r.addTargetSurface([5,5,0.01],[0;0;0],[0;0;0]);
% %r.addRobotFromURDF(r,'plate.urdf',zeros(3,1),zeros(3,1));
q1 = [0;0.2;1;0;0;0;0;0];

kinsol = doKinematics(r,q1);
% %active_collision_options.body_idx = [2; 3];
% [ptA,ptB,normal,distance] = pairwiseClosestPoints(r,kinsol,1,2);
% 

 active_collision_options.body_idx = [1;3];
% active_collision_options.collision_groups
% 
 [phi,surf_norm,xA,xB,idxA,idxB] = r.collisionDetect(kinsol, ...
                                           false, ...
                                           active_collision_options);
                                  
 [force,B_mod] = r.force{1,1}.computeSpatialForce(r,q1,zeros(8,1));
 [force2,B_mod2] = r.force{1,1}.computeNonAffineForce(r,q1,zeros(8,1),0);
 v = r.constructVisualizer();