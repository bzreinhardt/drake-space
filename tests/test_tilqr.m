%Test drake TILQR
d1 = 0.1*[1/2^.5,-1/2^.5,0];
d2 = 0.1*[-1/2^.5,-1/2^.5,0];
d3 = 0.1*[0,-1,0];
d = [d1;d2;d3];

a1 = [0,0,1];
a2 = [0,0,1];
a = [a1;a2;a1];

p = PlanarInspector(a,d);
x0 = [0;0.15;0;0;0;0];
[c] = findLQR(p,x0);

xnoise = 0.01;
dxnoise = 0.001;

sys = feedback(p,c);
v = PlanarInspectVisualizer(p);
xtraj = simulate(sys,[0 4],x0+[0;0.0001;0;0;0;0]);
v.playback(xtraj);
