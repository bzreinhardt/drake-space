function cost = forceObjective(u, actuator, dim, sgn)
%Find the actuation that will maximize actuation in a given dimension for a
%given sign
force = actuator(u);
if sign(force(dim)) ~= sgn
    cost = 100000;
else
    cost = inv(force(dim))^2;
end