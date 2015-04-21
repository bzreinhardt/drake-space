function [a,d] = armAnglesToInspectorParams(angles,l)
%converts a set of arm angles to 2d inspector build parameters
if nargin < 2
    l = 0.1; %default l
end
a = [zeros(2,length(angles)); ones(2,length(angles))]; %axes all in +z
d = zeros(3,length(angles));
for i = 1:length(angles)
    d(:,i) = [l*cos(angles(i));l*sin(angles(i));0];
    
end
end