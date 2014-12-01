%% Symbolic dynamics for an eddy current actuator 
%%Planar dynamics
%COM
x = sym('x');  
y = sym('y'); 
z = sym('z'); 

a1 = sym('a1');
a2 = sym('a2');

d1 = sym('d1');
d2 = sym('d2');


r = sym('r');
c1 = sym('c1');
c2 = sym('c2');

a = [a1;a2];
d = [d1;d2];




%% Full 6-DOF dynamics (later)
% a3
% rho = sym('rho');
% phi = sym('phi');
% psi = sym('psi');
% d3
% phi_surf
% psi_surf