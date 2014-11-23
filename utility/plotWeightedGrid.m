function plotWeightedGrid(grid, edge_size,weight)
%UNTITLED18 Summary of this function goes here
%   Detailed explanation goes here
% Horizontal grid 
ll_x = grid{1}-edge_size;
ll_z = grid{2}-edge_size;
cc = hsv(12);
plot(0,0,'Color',cc(1,:)); hold on;
plot(0,0,'Color',cc(2,:));
plot(0,0,'Color',cc(3,:));
plot(0,0,'Color',cc(4,:));
plot(0,0,'Color',cc(5,:));
plot(0,0,'Color',cc(6,:));
plot(0,0,'Color',cc(7,:));
str ={'0 actuation directions',...
    '1 actuation directions',...
    '2 actuation directions',...
    '3 actuation directions',...
    '4 actuation directions',...
    '5 actuation directions',...
    '6 actuation directions'};
for i = 1:numel(ll_x)
    
    rectangle('position',[ll_x(i),ll_z(i),edge_size,edge_size], ...
        'FaceColor',cc(ceil(weight(i)*6)+1,:));
end
hold off;
legend(str);

end

