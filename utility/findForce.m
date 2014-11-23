function [F,F_net] = findForce(w,a_world,g,v,surf_norm,f_x_spline,f_y_spline)
%Find force for N identical actuators
%   Vectorized, kinematics agnostic
% @param w - control speed - TODO  need to accept NxM 
% @param a - axis in world coordinates Nx3
% @param g - gap to surface - Nx1
% @param surf_norm - surface normal in world coordinates Nx3
% @param f_x_spline - ppspline force fn in the plate x direction
% @param f_y_spline - ppspline force fn in the plate y direction



dir_x = cross(a_world, surf_norm); %plate x direction in world frame
dir_y = cross(surf_norm,dir_x);%plate y direction in world frame

y_plate = surf_norm;
x_plate = cross(a_world, surf_norm);
norm_x_plate = sqrt(sum(x_plate.^2,2))*ones(1,3);
if sum(~any(norm_x_plate,2)) ~= 0
    norm_x_plate(~any(norm_x_plate,2),:) = [1 1 1];
end
x_plate = x_plate./norm_x_plate;
z_plate = cross(x_plate,y_plate);
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
if nargout > 1
    F_net = sum(reshape(F',3*m,n),2);
    F_net = reshape(F_net,3,m);
end
end

