function [ force_dir ] = genForceDir(obj, axis, n_hat, type )
%GENFORCEDIR finds the force between an induction coupler and a surface
% @param n_hat - surface normal in global coordinates
% @param axis - coupler axis in world frame
% @param g - distance between coupler and surface
%TODO: implement scaling from physics
%get forces in the frame of the surface
if nargin < 4
    type = 'PM';
end
%Find scaling factors from actual physics
f_x = 1;
f_y = 0; 
if strcmp(type,'PM')
dir_x = cross(axis,n_hat); 
dir_y = cross(dir_x,n_hat);

%axis along which force will act
force_dir = f_x*dir_x + f_y*dir_y;

end


end

