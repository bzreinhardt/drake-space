function g_max = findMaxEffectiveDistance(mean_noise,splines)
%FIND the maximum effective distance of a state dependent actuator 

% find the maximum effective distance of the actuators
    function cost = objective(g)
        f_x = fnval(splines.x,[0;0;g;10000]);
        f_y = fnval(splines.y,[0;0;g;10000]);
        cost = f_x^2+f_y^2 - mean_noise;
    end

Problem.x0 = 1;
Problem.objective = @(g)objective(g);
Problem.Aeq = [];
Problem.beq = [];
Problem.Aineq = [];
Problem.bineq = [];
%limits on u
Problem.lb = [0];
Problem.ub= [5];
Problem.options = optimset();
Problem.solver = 'fmincon';
% minimize difference between the generated force and the actual force
g_max = fmincon(Problem);


end

