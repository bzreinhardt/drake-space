function cost = objective(u,fStar,actuator)

% u = matrix of [2 x n] input states
% F = [3 x n] vector of force space outputs
% fStar = [3 x 1] vector of force space target
% actuator - function handle that gives force based on u

F = actuator(u);

n = size(u,2);
cost = sum((F-fStar*ones(1,n)).^2);

end