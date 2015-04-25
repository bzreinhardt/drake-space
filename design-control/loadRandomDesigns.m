%Load output from random intiial conditions
opts.folder_template = 'rand_output_';
opts.runs = 1:10;
loader = DesignLoader(opts);
[params, vol, gen] = loader.loadInspectorDesigns();
init_conds = loader.loadInitialConditions();
drawer = FigureGenerator;
h_main = figure(1340);
for i = 1:numel(params)
    h = drawer.drawDesign(params{i});
    
    drawer = drawer.set('fig_num',drawer.fig_num + 1);
    title_str = ['theta_0 = ',...
        mat2str(reshape(init_conds{i},1,numel(init_conds{i})),3)];
    title(title_str);
    h_temp = subplot(1,3,i,'Parent',h_main);
    new_pos = get(h_temp,'Position');
    delete(h_temp);
    set(h,'Parent',h_main,'Position',new_pos);
end
save2pdf('~/Documents/papers/controller_creation/graphics/three_initial_conditions_cmaes.pdf', h_main);