function testAvoidObstacleGpops(x0)
d1 = 0.1*[1/2^.5,-1/2^.5,0];
d2 = 0.1*[-1/2^.5,-1/2^.5,0];
d3 =  0.05*[0,-1,0];
d = [d1;d2;d3];

a1 = [0,0,1];
a2 = [0,0,1];
a3=a2;
a = [a1;a2;a3];

p = PlanarInspector(a,d);
output = avoidObstacleGpops(p,x0);
opts.p = p;

trajectory_plot(output,opts);
end