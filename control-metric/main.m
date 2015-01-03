%%%% Pseudo-code overview of control metric %%%%
%Initialize volume tracker
%Get a random point from stable region
%Stable region will be some subset of [x,y,theta], all v = 0
%Check if point is already in a controlled region 
%If not in controlled region, generate a controller
%Generate a lyapunov function for that system
%Check lyapunov function against state and input constraints, modify level
%set 
%mark the points in that region as controlled
%first pass - use 6D sparse matrix to represent space. each node should
%mark which controllers create regions that point to it
%second pass - use kd tree 
%Find if function overlaps any other regions and mark as neighbors
%generate volume based on number of points in controlled regions 
