function F = actuator1(u1)

%Example of simple eddy-current actuator, for two inputs.



F = [atan(4*u1);u1.^2;0*u1];


end