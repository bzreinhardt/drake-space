function [rpy] = findVisualrpy( axis )
%FINDVISUALRPY finds the roll pitch yaw to align the z axis of a cylindar
%with an axis
if numel(axis)~=3
    error('axis needs 3 elements');
end
if size(axis,1) ~= 3
    axis = axis';
end
axis = axis/norm(axis);
if isequal(axis,[0;0;1])
    rpy = [0;0;0];
    return
end
z = axis;
x = cross([0;0;1],axis);
y = cross(z,x);
w_R_b = [x y z];
rpy = rotmat2rpy(w_R_b);

end

