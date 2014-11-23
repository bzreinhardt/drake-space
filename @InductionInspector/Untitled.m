function [F,T] = findForceTorque(X,d,w,a,r,c,f_x_spline,f_y_spline)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% @param X - full state of object (xyzrpy,derivatives) 12x1
if size(X,2) ~= 1
    X = X';
end
if size(c,2) ~= 1
    c = c';
end
R = X(1:3)-c;
surf_norm = R/norm(R);
g = norm(R)-r;
w_R_b = rpy2rotmat(X(4:6));
a_world = w_R_b*a;

dir_x = cross(a_world, surf_norm); %plate x direction in world frame

dir_y = cross(dir_x, surf_norm);%plate y direction in world frame
if norm(dir_x) == 0
    warning('spin axis perpendicular to surface');
    F = zeros(3,1);
    T = zeros(3,1);
    return;
else
    y_plate = surf_norm;
    x_plate = cross(axis_world, surf_norm);
    x_plate = x_plate/norm(x_plate);
    z_plate = cross(x_plate,y_plate);
    w_R_p = [x_plate,y_plate,z_plate];
end

v_plate = w_R_p'*X(7:9);
f1 = fnval(f_x_spline,{v_plate(1),v_plate(2),g,w});
f2 = fnval(f_y_spline,{v_plate(1),v_plate(2),g,w});
f_w = dir_x*f1 + dir_y*f2; %force in the world frame




end

