function F = actuator(u)

%Example of simple eddy-current actuator, for two inputs.

u1 = u(1,:);
u2 = u(2,:);

f1 = [atan(4*u1);u1.^2;0*u1];
f2 = [0*u2;atan(4*u2);u2.^2];
F = f1+f2;

end