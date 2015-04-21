function [params, vol, gen] = loadPendulumDesigns(runs)
    if nargin < 1
        runs = [1:3,4:2:10];
    end
    vol = [];
    gen = [];
    params = {};
    for i = 1:length(runs)
        foldername = ['max_iterations_',num2str(runs(i))];
        disp(['loading ',foldername]);
        es = load([foldername, '/variablescmaes.mat']);
        if es.out.solutions.bestever.f ~= -10000
            vol(end+1) = -es.out.solutions.bestever.f;
            gen(end+1) = runs(i);
            params{end+1} = es.out.solutions.bestever.x;
        end
    end
end