%%DEFUNCT
clear all
r = InductionInspector('two_coupler_inspector.urdf');
r = r.addObstacles(1);
q = [0;0;1;0;0;0];
kinsol = r.doKinematics(q);

r = r.addLinksToCollisionFilterGroup('coupler1','coupler1',2);
r = r.addLinksToCollisionFilterGroup('world','coupler1',1);
active_collision_options.bodyidx = [1,2];
active_collision_options.collision_groups = {'coupler1'};
[distance,normal,xA,xB,idxA,idxB] = collisionDetect(r,kinsol,false,active_collision_options);