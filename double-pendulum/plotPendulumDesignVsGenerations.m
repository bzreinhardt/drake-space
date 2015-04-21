  function plotPendulumDesignVsGenerations()
  
            %plot inspector design convergence as you use more generations
            [params, vol, gen] = loadPendulumDesigns();
            plot(gen,vol,'x','MarkerSize',15);
            xlabel('Maximum Generations');
            ylabel('Controllable Volume');  
        end