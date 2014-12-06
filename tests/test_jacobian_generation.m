%test generating Jacobians

%Test drake TILQR
d1 = 0.1*[1/2^.5,-1/2^.5,0];
d2 = 0.1*[-1/2^.5,-1/2^.5,0];
d3 = 0.07*[0,-1,0];
d = [d1;d2;d3];

a1 = [0,0,1];
a2 = [0,0,1];
a = [a1;a2;a2];

p = PlanarInspector(a,d);
generateGradients(p);