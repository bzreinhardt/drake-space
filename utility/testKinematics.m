function [x,v,a_world] = testKinematics(X,a,d)
% Vectorized forward kinematics for an induction coupler
% X 12x1 state matrix
% a = axes in body coordinate Nx3
% d = coupler positions in body coordinate Nx3
w_R_b = rpy2rotmat(X(4:6));
T = [w_R_b X(1:3); 0 0 0 1];
a_world = (w_R_b*a')';
x = (T*[d';ones(1,size(d,1))]);
x = x(1:end-1,:)';
v = (w_R_b*X(7:9)*ones(1,size(d,1))+ cross(X(10:12)*ones(1,size(d,1)),d',1))';
end