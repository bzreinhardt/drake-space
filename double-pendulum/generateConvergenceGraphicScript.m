cd '~/drake-space/double-pendulum'
%demonstrateConvergenceCmaes;

baseline_volume = -min(loadjson('optimal_pend_vols.json')); 
        


fig = figure(1337);
clf;
plotPendulumDesignVsGenerations;
hold on
x_lims = get(gca,'XLim');
plot(x_lims,baseline_volume*ones(2,1), 'LineWidth',2);
hold off;
save2pdf('~/Documents/papers/controller_creation/graphics/cmaes_convergence.pdf');
