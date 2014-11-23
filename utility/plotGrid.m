function plotWeightedGrid(grid, edge_size,weight)
%UNTITLED18 Summary of this function goes here
%   Detailed explanation goes here
% Horizontal grid 
ll_x = grid{1}-edge_size;
ll_z = grid{2}-edge_size;
cc = hsv(12);
for i = 1:numel(ll_x)
    rectangle('position',[ll_x(i),ll_z(i),edge_size,edge_size], ...
        'FaceColor',cc(ceil(weight(i)*6),:));
end



end

