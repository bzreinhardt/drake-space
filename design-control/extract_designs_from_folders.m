%Extract useful info from log files
runs = 2:20;
f_max = [];
generations_max = [];
param_max = {};
for i = 1:length(runs)
    foldername = ['output_',num2str(runs(i))];
    disp(['loading ',foldername]);
    es = load([foldername, '/variablescmaes.mat']);
    if es.out.solutions.bestever.f ~= -10000
    f_max(end+1) = -es.out.solutions.bestever.f;
    generations_max(end+1) = runs(i);
    param_max{end+1} = es.out.solutions.bestever.x;
    end
end
figure(1337);clf;
plot(generations_max,f_max,'x');
xlabel('Maximum Generations');
ylabel('Controllable Volume');
    
    