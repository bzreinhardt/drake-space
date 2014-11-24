function h = drawInductionInspector2D(state,options)
x = state(1);
y = state(2);
theta = state(3);
if nargin <2
    options = 0;
end

w_R_b = [cos(theta) -sin(theta); sin(theta) cos(theta)];
T = [w_R_b, [x;y]; 0 0 1];

corners_x = 0.05*[-1 -1 1 1];
corners_y = 0.05*[-1 1 1 -1];

box = T*[corners_x;corners_y; ones(1,4)];

h = patch(box(1,:),box(2,:),[200 200 200]/255);

end