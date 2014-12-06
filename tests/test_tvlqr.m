%test tvlqr
d1 = 0.1*[1/2^.5,-1/2^.5,0];
d2 = 0.1*[-1/2^.5,-1/2^.5,0];
d3 = 0.07*[0,-1,0];
d = [d1;d2;d3];

a1 = [0,0,1];
a2 = [0,0,1];
a = [a1;a2;a2];

p = PlanarInspector(a,d);

%load('~/drake-space/trajectories-2d/sample_trajectory.mat');
load('~/drake-space/trajectories-2d/drake_test_trajectory.mat');
x0 = eval(xtraj,0);
[c] = findTVLQR(p,xtraj,utraj);
sys = feedback(p,c);
xtraj_sim = simulate(sys,[0 40],x0);
figure(100); clf;
subplot(211);

fnplt(xtraj,1);
hold on;
fnplt(xtraj_sim,1);
hold off;
subplot(212)
fnplt(xtraj,2);
hold on;
fnplt(xtraj_sim,2);
hold off;