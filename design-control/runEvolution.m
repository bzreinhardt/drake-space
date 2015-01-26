function [best_design, max_fitness] = runEvolution(fit_fun, create_fun, options)
if nargin < 3
    options = struct();
end
%set up default options
if ~isfield(options,'method')
    options.method = 'random';
end
if ~isfield(options,'max_iterations')
    options.max_iterations = 300;
end
if ~isfield(options, 'max_no_improve')
    options.max_no_improve = 25;
end
if strcmp(options.method, 'random')
    best_design = [];
    max_fitness = 0;
    iterations = 0;
    no_improve = 0;
    %keep track of how many iterations have happened without fitness getting
    %better
    while(iterations < options.max_iterations && ...
            no_improve < options.max_no_improve)
        fprintf('running iteration %d \n',iterations);
        fprintf('no improvement for %d iterations \n', no_improve);
        %generate
        design = feval(create_fun);
        %find fitness
        fitness = feval(fit_fun,design);
        if fitness > max_fitness
            max_fitness = fitness;
            best_design = design;
            no_improve = 0;
        else
            no_improve = no_improve + 1;
        end
        iterations = iterations + 1;
        
    end
    if iterations >= options.max_iterations
        disp('RUNEVOLUTION: exit - exceed max iterations');
    elseif no_improve >= options.max_no_improve
        disp('RUNEVOLUTION: exit - exceed no_improve');
    end
    
end
end