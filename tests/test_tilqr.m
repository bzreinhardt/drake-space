

%Test drake TILQR
d1 = 0.1*[1/2^.5,-1/2^.5,0];
d2 = 0.1*[-1/2^.5,-1/2^.5,0];
d3 = 0.07*[0,-1,0];
d = [d1;d2;d3];

a1 = [0,0,1];
a2 = [0,0,1];
a = [a1;a2;a2];

p = PlanarInspector(a,d);

%%%%%%%%%% Some sample optimal control and actual position data %%%%%%%%
optimal = [0.0002;  0.1375;  -0.0000;   0.0000;  -0.0010; -0.0000];
optimal_u = [-113.5476; 113.3777;-0.3021];
optimal(4:6) = zeros(3,1);

real = [0.0005; 0.1457; -0.0000; 0.0001; -0.0004;  -0.0000];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x0 = optimal;
[c] = findLQR(p,x0);

xnoise = 0.01;
dxnoise = 0.001;

sys = feedback(p,c);
v = PlanarInspectVisualizer(p);

xtraj = simulate(sys,[0 100],real);
clear sys;
figure(99); clf;
subplot(211);

fnplt(xtraj,2);
hold on;
fnplt(xtraj,1);
hold off;
subplot(212)
fnplt(xtraj,4);
hold on;
fnplt(xtraj,5);
hold off;

%v.playback(xtraj);

%%%%%%%%%%%%%%%%%%%%%
