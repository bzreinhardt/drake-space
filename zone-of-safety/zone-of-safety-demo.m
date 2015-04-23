%possible algorithm for knowing whether a point is in the zone of safety

%A) just do an optimization problem with the constraint being that it can't
%intersect the surface
%B) gradient heuristic - find max possible gradient towards/away from
%target for a series of time steps in the simulation
%C) Fire all thrusters 

d1 = 0.1*[1/2^.5,-1/2^.5,0];
d2 = 0.1*[-1/2^.5,-1/2^.5,0];
d3 =  0.05*[0,-1,0];
d = [d1;d2;d3];

a1 = [0,0,1];
a2 = [0,0,1];
a3=a2;
a = [a1;a2;a3];

p = PlanarInspector(a,d);
