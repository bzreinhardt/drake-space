function demonstrateConvergenceCmaes
%%%%%%%%%%% Pendulum parameters %%%%%%%
mass_range = [0.5 1.5];
length_range = [0.5 1.5];
opts.LBounds = [mass_range(1)*ones(2,1);length_range(1)*ones(2,1)];
opts.UBounds = [mass_range(2)*ones(2,1);length_range(2)*ones(2,1)];
opts.LogPlot = 'on';

%Genetic Algorithm parameters
X0 = [mean(length_range)*ones(2,1);mean(mass_range)*ones(2,1)];
sigma = 0.5*ones(4,1);
N =10; %number of iterations
%run cmaes for some number of iterations
for its = 1:3
    opts.StopIter = its;
    [XMIN, FMIN, COUNTEVAL, STOPFLAG, OUT, BESTEVER] = cmaes('pendulumFitness',X0,sigma,opts);
    folder = strcat('max_iterations_',num2str(its));
    system(['bash rename_cmaes.sh ',folder]);
end
end