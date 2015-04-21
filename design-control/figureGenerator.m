classdef figureGenerator
    properties
        target_folder = '~/Documents/papers/controller_creation/graphics/';
        fig_folder = '~/drake-space/graphics/control-metric-design/';
        fig_array = [1330,1331,1332,1333];
        name_array = {'d_human','d_algorithm','c_human',...
                'c_algorithm'};
            
        
    end
    %Knepper figure fixes:
    % Early figure for paper - make interactive opengl thing?
    % Normalize figure 2 
    % Zoom in on figure 3c-d
    methods
        
        function randPendConvergenceFig(obj,big)
            %convergence for pendulum using random
            %default: big
            [volumes,iterations,baseline] = parsePendTests('/home/ben/drake-space/data/pendulum_convergence_test/long_convergence_test');
            figure(1234); clf;
            
            scatter(iterations,volumes,50,'*'); hold on;
            if nargin < 2 || big == 1
            plot([0,iterations,max(iterations)+50],baseline.controller.volume*ones(size([0,iterations])+1),'LineWidth',3,'Color','b');
            legend('Generated Controllers','Optimal Design');
            xlabel('Number of Iterations');
            ylabel('Normalized Controller Volume');
            ylim([0.1, 1.3]);
            xlim([0, max(iterations)+50]);
            
            hgsave(strcat(obj.fig_folder,'pendulum-convergence-big.fig'));
            save2pdf(strcat(obj.target_folder,'pendulum-convergence-bg.pdf'));
            else
                 plot([0,iterations],baseline.controller.volume*ones(size([0,iterations])),'LineWidth',3);
            legend('Generated Controllers','Optimal Design');
            xlabel('Number of Iterations');
            ylabel('Normalized Controller Volume');
            ylim([0.1, 1.3]);
            
            hgsave(strcat(obj.fig_folder,'pendulum-convergence.fig'));
            save2pdf(strcat(obj.target_folder,'pendulum-convergence.pdf'));
            end
                
        end
        
        function generateInspectorDesignVsGenerations(obj)
            %plot inspector design convergence as you use more generations
            [params, vol, gen] = loadInspectorDesigns();
            figure(1337);clf;
            plot(gen,vol,'x');
            xlabel('Maximum Generations');
            ylabel('Controllable Volume');
            
        end
        
        function generateAlgControlVolumeFig(obj)
            %generates control volume and image for best design from cmaes
            figure(obj.fig_array(3));clf;
            [params, vol, gen] = loadInspectorDesigns();
            [a,d] = armAnglesToInspectorParams(params{end});
            insp = Inspector2d(a,d);
            defaults = loadjson('inspector_default_values.json');
            range = defaults.range;
            opt_prm = LQRPRM(insp,range);
            opt_prm.regions_max = 20;
            options = struct;
            options.method = 'tilqr';
            options.rand_method = 'rand';
            opt_prm = opt_prm.fillRegion(options);
            opt_prm.draw(obj.fig_array(3));
            title(['V_a = ', num2str(opt_prm.volume)]);
            xlabel('x1 (m)');
            ylabel('x2 (m)');
           
        end
        
        function generateAlgDesignFig(obj)
            figure(obj.fig_array(2));clf;
            %generates figure for the best design
            [params, vol, gen] = loadInspectorDesigns();
            [a,d] = armAnglesToInspectorParams(params{end});
            insp = Inspector2d(a,d);
            
 
insp_vis = Inspector2dVisualizer(insp,obj.fig_array(2));
           insp_vis.draw(0,[0;0.11;zeros(4,1)]);
           xlabel('x1 (m)');
            ylabel('x2 (m)');
            title('');
        end
        
        function generateHumanControlVolumeFig(obj)
            fig_num = obj.fig_array(4);
            figure(fig_num);clf;
             %generates control volume and image for human design
            baseline = loadjson('human_baseline_vals.json');
            a = baseline.a;
            d = baseline.d;
            insp = Inspector2d(a,d);
            defaults = loadjson('inspector_default_values.json');
            range = defaults.range;
            opt_prm = LQRPRM(insp,range);
            opt_prm.regions_max = 20;
            options = struct;
            options.method = 'tilqr';
            options.rand_method = 'rand';
            opt_prm = opt_prm.fillRegion(options);
            opt_prm.draw(fig_num);
            
            title(['V_h = ', num2str(opt_prm.volume)]);
            xlabel('x1 (m)');
            ylabel('x2 (m)');
        end
        
        function generateHumanDesignFig(obj)
            fig_num = obj.fig_array(1);
            figure(fig_num);clf;
               %generates figure for the human design
            baseline = loadjson('human_baseline_vals.json');
            a = baseline.a;
            d = baseline.d;
            insp = Inspector2d(a,d);
           
           insp_vis = Inspector2dVisualizer(insp,fig_num);
           insp_vis.draw(0,[0;0.11;zeros(4,1)]);
           xlabel('x1 (m)');
            ylabel('x2 (m)');
        title('');
        end
        
        
        function obj = loadFourFigs(obj)
          %load the four algorithm pictures  
            obj.generateHumanDesignFig;
            obj.generateAlgDesignFig;
            obj.generateHumanControlVolumeFig;
            obj.generateAlgControlVolumeFig;
            
        end
        
        function saveFourFigsToPdf(obj,open)

            if nargin>1
                obj.loadFourFigs;
            end
            %generate baseline and algorithmic figures
            xl_d = [-0.2 0.2];
            yl_d = [0.0 0.2];
            xl_c = [-0.13 0.13];
            yl_c = [0.08 0.17];
            
            for i = 1:length(obj.name_array)
                figure(obj.fig_array(i))
                if i < 3
                    xl = xl_d;
                    yl = yl_d;
                else
                    xl = xl_c;
                    yl = yl_c;
                end
                    
                xlim(xl);
                ylim(yl);
                set(obj.fig_array(i),'Position',[100 100 800 600]);
                xlabel('x1 (m)'); ylabel('x2 (m)');
                %hgsave(strcat(obj.fig_folder,obj.name_array{i},'.fig'));
                save2pdf(strcat(obj.target_folder,obj.name_array{i},'.pdf'));
            end
        end
  %%%%%%% Functions for dealing with the simple regions graphics %%%%%%      
         function updateSimpleRegions(obj,filename,foldername)
            if nargin < 3
                foldername = '/home/ben/drake-space/graphics/control-metric-design/';
            end
            if nargin < 2
                filename = 'simple_regions.fig';
            end
            fig = hgload(strcat(foldername,filename));
            children = get(gca,'children');
            marker_children = children(isprop(children,'MarkerSize')&~isprop(children,'AmbientStrength'));
            text_children = children(isprop(children,'FontWeight'));
            set(marker_children,'MarkerSize',20);
            %color_children = children(isprop(children,'Color'));
           % set(marker_children,'Color','k');
            set(marker_children,'LineWidth',2');
            set(marker_children,'Color','k');
            text_positions = get(text_children,'Position');
            new_positions = cellfun(@(x)(x+[0 -0.003 0]),text_positions,'UniformOutput',false);
            for i = 1:numel(text_children)
           
            set(text_children(i),'Position', new_positions{i});
            end
            save2pdf(strcat(obj.target_folder,'simple_regions.pdf'));
            
         end
        
         function updateSimpleGraph(obj,filename,foldername)
            if nargin < 3
                foldername = '/home/ben/drake-space/graphics/control-metric-design/';
            end
            if nargin < 2
                filename = 'simple_graph_and_path.fig';
            end
            fig = hgload(strcat(foldername,filename));
            axis off;
             children = get(gca,'children');
            text_children = children(isprop(children,'FontWeight'));
            text_positions = get(text_children,'Position');
            new_positions = cellfun(@(x)(x+[0 -0.002 0]),text_positions,'UniformOutput',false);
            for i = 1:numel(text_children)
           
            set(text_children(i),'Position', new_positions{i});
            end
           % set(gca,'BackgroundColor','w');
            save2pdf(strcat(obj.target_folder,'simple_graph_and_path.pdf'));
         end
             
    end
    
    
    methods(Static)
        %% Update simple path figures
       
        
        
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