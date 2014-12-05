%testing gpops2DrakeSpline
megaclear;
p = PlanarInspector;
load('sample_gpops_output.mat');

output.phase = solution;

[xtraj, utraj] = gpops2DrakeSpline(p,output);
[ytraj_sim,xtraj_sim] = p.runOL(utraj);

