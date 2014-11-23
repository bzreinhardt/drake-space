function lims = findActuatorPotential(q,params,world)
a = params.a;
d = params.d;
f_x_spline = params.spline.x;
f_y_spline = params.spline.y;
r = world.r;
c = world.c;
[x,v,a_world] = testKinematics(q,a,d);
[g, surf_norm] = sphereNormGap(x, r, c);
actuator = @(u)findNetForce(u,a_world,g,v,surf_norm,f_x_spline,f_y_spline);
% create a long array of actuator possibilities. brute force way to find
% the actuation potential at a position
n = 25;
u_min = -1E4;
u_max = -u_min; 
num_u = size(a,1);

ui = repmat(linspace(u_min,u_max,n),num_u,1);
u_cell = num2cell(ui,2);
u_cell = u_cell';
Ui = cell(1,num_u);
[Ui{:}] = meshgrid(u_cell{:});
flatten = @(x)reshape(x,1,n^num_u);
Ui = cellfun(flatten,Ui,'UniformOutput',false);
Ui = Ui';
u = cell2mat(Ui);
if g > 0
    f = actuator(u);
else 
    f = zeros(3,size(u,2));
end

[fx_max,fx_max_idx] = max(f(1,:));
[fy_max,fx_max_idx] = max(f(2,:));
[fz_max,fx_max_idx] = max(f(3,:));
[fx_min,fx_max_idx] = min(f(1,:));
[fy_min,fx_max_idx] = min(f(2,:));
[fz_min,fx_max_idx] = min(f(3,:));

lims = [fx_min fx_max;...
    fy_min fy_max
    fz_min fz_max];

end