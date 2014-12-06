%% test gpops sim --> drake stack
x0 = [0.0;0.15;0;0;0;0];
xf = [0.0;0.10;0;0;0;0];
d1 = 0.1*[1/2^.5,-1/2^.5,0];
d2 = 0.1*[-1/2^.5,-1/2^.5,0];
d3 =  0.05*[0,-1,0];
d = [d1;d2;d3];

a1 = [0,0,1];
a2 = [0,0,1];
a3=a2;
a = [a1;a2;a3];


p = PlanarInspector(a,d);
options.x0 = x0; options.xf = xf;
simulation = @()dircolGpops(p,options);

gpops_output = feval(simulation);

trajectory_plot(gpops_output);


[xtraj, utraj] = gpops2DrakeSpline(p,gpops_output);
[ytraj_sim,xtraj_sim] = runOL(p,utraj,x0);
figure(1);hold on; fnplt(xtraj_sim,1);hold off;
figure(2);hold on; fnplt(xtraj_sim,2);hold off;