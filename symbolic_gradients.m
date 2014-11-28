%2D things
x = sym('x');
y = sym('y');
theta = sym('theta');
vx = sym('vx');
vy = sym('vy');
omega = sym('omega');
X = [x y theta vx vy omega];
cx = sym('cx');
cy = sym('cy');

r = sym('r');

dx = sym('dx');
dy = sym('dy');

dFxdg = sym('dFxdg');
dFxdvtan = sym('dFxdvx');
dFxdvperp = sym('dFxdvy');
dFxdu = sym('dFxdu');

dFydg = sym('dFydg');
dFydvtan = sym('dFydvx');
dFydvperp = sym('dFydvy');
dFydu = sym('dFydu');

u = sym('u');

Fx = [dFxdg*g + dFxdu*u];
Fy = [dFydg*g + dFydu*u];

v = [vx;vy];
c = [cx; cy];
d_b = [dx;dy];
d_w = [cos(theta) -sin(theta); sin(theta) cos(theta)]*d_b + [x;y];

g = sqrt(norm((d_w - c))^2 - r^2);

n = (d_w-c)/norm(d_w-c);




dd = jacobian(d_w, X);
dg = simplify(jacobian(g, X));
dn = jacobian(n);

F = [-Fx*n(2) + Fy*n(1); ...
    Fx*n(1) + Fy*n(2); ...
    -d_w(2)*(-Fx*n(2)+Fy*n(1))+d_w(1)*(Fx*n(1) + Fy*n(2))];

dFdx = jacobian(F,X);
dFdu = jacobian(F,u);
    
    






