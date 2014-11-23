function F = actuator2(u2)

%Example of simple eddy-current actuator, for two inputs.



F = [atan(4*u2);u1.^2;0*u2];


end