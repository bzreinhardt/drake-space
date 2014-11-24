function [g, surf_norm] = sphereNormGap(x, r, c)
% Find the gap between an actuator and the closest point on a sphere and
% its normal
%TODO vectorize
% @param x Nx3 position in world coordinates
if size(c,2) ~= 1
    c = c';
end
states = 3;
if (size(x,2) == 2)
    %assume you're in 2D
    states = 2;
    c = c(1:states);
end
    
R = x(:,1:states)-ones(size(x,1),1)*c';
surf_norm = R./(sqrt(sum(R.^2,2))*ones(1,states));

g = sqrt(sum(R.^2,2))-r;
if any(g<=0)
    error_msg = strcat('sphereNormGap: coupler is inside the freaking sphere');
    warning(error_msg);
end