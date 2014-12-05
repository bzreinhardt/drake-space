%% test gpops sim --> drake stack
x0 = [0.05;0.2;0;0;0;0];
xf = [0.05;0.15;0;0;0;0];
d1 = 0.1*[1/2^.5,-1/2^.5,0];
d2 = 0.1*[-1/2^.5,-1/2^.5,0];
d3 = 0.1*[0,-1,0];
d = [d1;d2;d3];

a1 = [0,0,1];
a2 = [0,0,1];
a = [a1;a2;a1];


p = PlanarInspector(a,d);
options.x0 = x0; options.xf = xf;
gpops_output = dircolGpops(p,options);
trajectory_plot(gpops_output);


[xtraj, utraj] = gpops2DrakeSpline(p,gpops_output);
[ytraj_sim,xtraj_sim] = runOL(p,utraj,x0);