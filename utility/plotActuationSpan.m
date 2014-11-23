function plotActuationSpan(actuator, num_u)
%Plots the possible forces a set of actuators can create
%@param actuator - function handle that takes NxM inputs
%@param number of actuators
n = 25;
u_min = -1E4;
u_max = -u_min; 

ui = repmat(linspace(u_min,u_max,n),num_u,1);
u_cell = num2cell(ui,2);
u_cell = u_cell';
Ui = cell(1,num_u);
[Ui{:}] = meshgrid(u_cell{:});
flatten = @(x)reshape(x,1,n^num_u);
Ui = cellfun(flatten,Ui,'UniformOutput',false);
Ui = Ui';
u = cell2mat(Ui);

f = actuator(u);
scatter3(f(1,:),f(2,:),f(3,:));


end
