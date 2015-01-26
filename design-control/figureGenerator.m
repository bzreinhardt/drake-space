classdef figureGenerator
    methods(Static)
        %% Generate an example path planning figure
        %Two different figures, nodes and actual regions 
        function genSampleRegionAndGraph
        sys = Inspector2d;
    x_range = [-0.05 0.05];
    y_range = [0.10 0.12];
    theta_range = [0 0];
    dx_range = [0 0];
    dy_range = [0 0];
    dtheta_range = [0 0];
    range = [x_range; y_range; theta_range; dx_range; dy_range; dtheta_range];
    x0 = [[-0.03;0.11; zeros(4,1)],[0.01;0.107; zeros(4,1)],...
        [0.03;0.113; zeros(4,1)], [0.00;0.122; zeros(4,1)],[0.06;0.109; zeros(4,1)]];
    prm = LQRPRM(sys,range);
    options = struct();
    options.method = 'tilqr';
    options.normalize = true;
    

    figure(1337);clf;
    for i=1:size(x0,2)
        options.x0 = x0(:,i);
        prm = prm.genControlRegion(options);
        prm = prm.findVolume(options);

    end
    fprintf('\n total volume = %f \n',prm.volume);
    disp(prm.occupancy_map);
    figure(1337);clf;
    options.dims = [1,2];
    
    options.label = true;
  
    options.color = 'rand';
   options.edge_color = true;
    prm.draw(gcf,options);
    figure(1338);clf;
    prm.plotGraph;
        end
        
        
    end
end