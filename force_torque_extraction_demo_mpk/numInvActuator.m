function [u_soln, f_soln, f_guess,err] = numInvActuator(num_u,f_target,actuator)
% numerically invert an actuator system to find the u that will generate
% the desired f

%parameters that can be changed later
n = 25;
u_max = 1E4;
u_min = -u_max;
%grid of possible inputs

ui = repmat(linspace(u_min,u_max,n),num_u,1);
u_cell = num2cell(ui,2);
u_cell = u_cell';
Ui = cell(1,num_u);
[Ui{:}] = meshgrid(u_cell{:});
flatten = @(x)reshape(x,1,n^num_u);
Ui = cellfun(flatten,Ui,'UniformOutput',false);
Ui = Ui';
u = cell2mat(Ui);


%cost function for the optimization
cost = objective(u,f_target,actuator);
%grid search to find the first guess
[~,idx] = min(cost,[],2);
%closest u in u grid
uGuess = u(:,idx);
% foce generated by closest u
f_guess = actuator(uGuess);

u_soln = zeros(size(uGuess));
f_soln = zeros(size(f_guess));

Problem.x0 = uGuess;
Problem.objective = @(u)objective(u,f_target,actuator);
Problem.Aeq = [];
Problem.beq = [];
Problem.Aineq = [];
Problem.bineq = [];
%limits on u
Problem.lb = u_min*ones(num_u,1);
Problem.ub= u_max*ones(num_u,1);
Problem.options = optimset();
Problem.solver = 'fmincon';
% minimize difference between the generated force and the actual force
u_soln(:) = fmincon(Problem);
f_soln(:) = actuator(u_soln);

%% Generate error
if nargout > 3
    err = f_target-fSoln;
    norm_err = norm(fTarget-fSoln);
    err_max = 0.1;
    if norm_err > err_max
        warning('Cannot find control close enough to desired control');
    end
end

end

