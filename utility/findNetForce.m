function [f_net] = findNetForce(w,a_world,g,v,surf_norm,f_x_spline,f_y_spline)
%Find net force for N identical actuators, with m different inputs
%   Vectorized, kinematics agnostic
% @param w - control speed - TODO  need to accept NxM 
% @param a - axis in world coordinates Nx3
% @param g - gap to surface - Nx1
% @param surf_norm - surface normal in world coordinates Nx3
% @param f_x_spline - ppspline force fn in the plate x direction
% @param f_y_spline - ppspline force fn in the plate y direction



dir_x = cross(a_world, surf_norm,2); %plate x direction in world frame
dir_y = cross(surf_norm,dir_x,2);%plate y direction in world frame

y_plate = surf_norm;
x_plate = cross(a_world, surf_norm,2);
norm_x_plate = sqrt(sum(x_plate.^2,2))*ones(1,3);
if sum(~any(norm_x_plate,2)) ~= 0
    norm_x_plate(~any(norm_x_plate,2),:) = [1 1 1];
end
x_plate = x_plate./norm_x_plate;
z_plate = cross(x_plate,y_plate,2);
%w_R_p = [x_plate;y_plate;z_plate];

v1_plate = x_plate(:,1)*v(1) + y_plate(:,1)*v(1) + z_plate(:,1)*v(1);
v2_plate = x_plate(:,2)*v(2) + y_plate(:,2)*v(2) + z_plate(:,2)*v(2);
%TODO vectorize in terms of w need to have 
[n,m] = size(w);

f1 = fnval(f_x_spline,[repmat(v1_plate',1,m);...
    repmat(v2_plate',1,m);...
    repmat(g',1,m);...
    reshape(w,1,n*m)]); 
f2  = fnval(f_y_spline,[repmat(v1_plate',1,m);...
    repmat(v2_plate',1,m);...
    repmat(g',1,m);...
    reshape(w,1,n*m)]); 

F = repmat(dir_x,m,1).*(f1'*ones(1,3))...
    + repmat(dir_y,m,1).*(f2'*ones(1,3)); %force in the world frame
f_net = zeros(3,m);
for i = 1:n
    f_net = f_net + F(i:n:end,:)';
   
end


end

