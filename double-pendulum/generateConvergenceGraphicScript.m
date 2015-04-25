cd '~/drake-space/double-pendulum'
%demonstrateConvergenceCmaes;

opt_vols = loadjson('optimal_vols_20_regions.json')
baseline_volume = -min(opt_vols.range); % 
        


fig = figure(1337);
clf;
 [params, vol, gen] = loadPendulumDesigns();
            plot(gen,vol/baseline_volume,'*','MarkerSize',15);
            xlabel('Maximum Generations');
            ylabel('Controllable Volume');  
hold on
x_lims = get(gca,'XLim');
plot(x_lims, ones(2,1), 'LineWidth',4);
hold off;
ylim([0;1.2]);
legend('Generated Controllers','Optimal Design','Location','NorthEast');
save2pdf('~/Documents/papers/controller_creation/graphics/cmaes_convergence.pdf');
