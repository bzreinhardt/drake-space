p = Inspector2d();
x_nom = [0;0.15;0;0;0;0];
%sanity checks for induction couplers, unit tests

%a single coupler with an axis perpendicular to a surface should have zero
%force
[xdot, dxdot] = p.dynamics(0,x_nom,[0;0;0]);
valuecheck(xdot,zeros(6,1));
sizecheck(dxdot, [p.getNumStates, 1 + p.getNumStates + p.getNumInputs]);



%there should be no force when the input is zero 

v = Inspector2dVisualizer(p);
v.draw(0,x_nom);


