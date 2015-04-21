function [params, vol, gen] = loadInspectorDesigns()
    runs = 3:20;
    vol = [];
    gen = [];
    params = {};
    for i = 1:length(runs)
        foldername = ['output_',num2str(runs(i))];
        disp(['loading ',foldername]);
        es = load([foldername, '/variablescmaes.mat']);
        if es.out.solutions.bestever.f ~= -10000
            vol(end+1) = -es.out.solutions.bestever.f;
            gen(end+1) = runs(i);
            params{end+1} = es.out.solutions.bestever.x;
        end
    end
end