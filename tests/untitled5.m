%Test drake TILQR
p = PlanarInspector;
x0 = [0.1;0.15;0;0;0;0];
[c,V] = findLQR(p,x0);

sys = feedback(p,c);
v = PlanarInspectVisualizer(p);
xtraj = simulate(sys,[0 4],x0);
v.playback(xtraj);
