function dxdot = planarCouplerGradient(t,X,u,options)
%PLANARCOUPLERGRADIENT
%    DXDOT = PLANARCOUPLERGRADIENT(C1,C2,D1,D2,R,THETA,U,X,Y)

%    This function was generated by the Symbolic Math Toolbox version 5.11.
%    25-Dec-2014 10:39:26

if nargin < 4
    d = options.d;
    radius = options.radius;
    center = options.center;
else
    d = [0;-1/sqrt(2);0];
    
    radius = 5;
    center = [0;-5];
end
c1 = center(1);
c2 = center(2);
d1 = d(1);
d2 =d(2);
r = radius;
theta = X(3);
x = X(1);
y = X(2);

t3 = cos(theta);
t4 = sin(theta);
t6 = d1.*t3;
t7 = d2.*t4;
t2 = c1-t6+t7-x;
t9 = d2.*t3;
t10 = d1.*t4;
t5 = -c2+t9+t10+y;
t8 = t2.^2;
t11 = t5.^2;
t12 = t8+t11;
t13 = u.^2;
t14 = sqrt(t12);
t15 = r.*4.41463e1;
t38 = t14.*4.41463e1;
t16 = t15-t38;
t17 = exp(t16);
t18 = u.*4.7160845866e-4;
t19 = t13.*3.844372009e-11;
t40 = t13.*u.*3.485746657e-13;
t20 = t18+t19-t40;
t21 = c1.*2.0;
t22 = d2.*t4.*2.0;
t28 = x.*2.0;
t29 = d1.*t3.*2.0;
t23 = t21+t22-t28-t29;
t24 = r.*3.89901e1;
t34 = t14.*3.89901e1;
t25 = t24-t34-2.18594e1;
t26 = exp(t25);
t27 = 1.0./t12;
t30 = t13.*6.301449255e1;
t31 = t13.*u.*7.66709574e-7;
t32 = t13.^2;
t36 = u.*1.0074136425e1;
t37 = t32.*1.964786265e-7;
t33 = t30+t31-t36-t37;
t35 = 1.0./t12.^(3.0./2.0);
t39 = 1.0./sqrt(t12);
t41 = y.*2.0;
t42 = d2.*t3.*2.0;
t43 = d1.*t4.*2.0;
t45 = c2.*2.0;
t44 = t41+t42+t43-t45;
t46 = t9+t10;
t47 = t6-t7;
t48 = t2.*t46.*2.0;
t49 = t5.*t47.*2.0;
t50 = t48+t49;
t51 = u.*1.260289851e2;
t52 = t13.*2.300128722e-6;
t74 = t13.*u.*7.85914506e-7;
t53 = t51+t52-t74-1.0074136425e1;
t54 = u.*7.688744017999999e-11;
t76 = t13.*1.0457239971e-12;
t55 = t54-t76+4.7160845866e-4;
t56 = t17.*t20.*t39;
t57 = t26.*t33.*t39;
t58 = t9+t10+y;
t59 = t5.*t17.*t20.*t27.*t44.*2.207315e1;
t60 = t5.*t17.*t20.*t35.*t44.*(1.0./2.0);
t61 = t6-t7+x;
t62 = t26.*t39.*(t30+t31-t36-t37);
t63 = t2.*t17.*t20.*t27.*t44.*2.207315e1;
t64 = t2.*t17.*t20.*t35.*t44.*(1.0./2.0);
t65 = t62+t63+t64-t5.*t26.*t27.*t33.*t44.*1.949505e1-t5.*t26.*t33.*t35.*t44.*(1.0./2.0);
t66 = t26.*t39.*t47.*(t30+t31-t36-t37);
t67 = t2.*t17.*t20.*t27.*t50.*2.207315e1;
t68 = t2.*t17.*t20.*t35.*t50.*(1.0./2.0);
t69 = t66+t67+t68-t17.*t20.*t39.*t46-t5.*t26.*t27.*t33.*t50.*1.949505e1-t5.*t26.*t33.*t35.*t50.*(1.0./2.0);
t70 = t5.*t17.*t20.*t27.*t50.*2.207315e1;
t71 = t5.*t17.*t20.*t35.*t50.*(1.0./2.0);
t72 = t5.*t17.*t20.*t39;
t73 = t2.*t26.*t39.*(t30+t31-t36-t37);
t75 = t5.*t26.*t39.*t53;
t77 = t75-t2.*t17.*t39.*t55;
dxdot = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t57-t5.*t17.*t20.*t23.*t27.*2.207315e1-t5.*t17.*t20.*t23.*t35.*(1.0./2.0)-t2.*t23.*t26.*t27.*t33.*1.949505e1-t2.*t23.*t26.*t33.*t35.*(1.0./2.0),t56+t5.*t23.*t26.*t27.*(t30+t31-t36-t37).*1.949505e1+t5.*t23.*t26.*t35.*(t30+t31-t36-t37).*(1.0./2.0)-t2.*t17.*t20.*t23.*t27.*2.207315e1-t2.*t17.*t20.*t23.*t35.*(1.0./2.0),t61.*(t56-t2.*t17.*t20.*t23.*t27.*2.207315e1-t2.*t17.*t20.*t23.*t35.*(1.0./2.0)+t5.*t23.*t26.*t27.*t33.*1.949505e1+t5.*t23.*t26.*t33.*t35.*(1.0./2.0))+t58.*(-t57+t2.*t23.*t26.*t27.*(t30+t31-t36-t37).*1.949505e1+t2.*t23.*t26.*t35.*(t30+t31-t36-t37).*(1.0./2.0)+t5.*t17.*t20.*t23.*t27.*2.207315e1+t5.*t17.*t20.*t23.*t35.*(1.0./2.0))-t2.*t17.*t20.*t39+t5.*t26.*t39.*(t30+t31-t36-t37),0.0,0.0,0.0,t59+t60-t17.*t20.*t39+t2.*t26.*t27.*t44.*(t30+t31-t36-t37).*1.949505e1+t2.*t26.*t35.*t44.*(t30+t31-t36-t37).*(1.0./2.0),t65,t72+t73-t58.*(-t56+t59+t60+t2.*t26.*t27.*t33.*t44.*1.949505e1+t2.*t26.*t33.*t35.*t44.*(1.0./2.0))+t61.*t65,0.0,0.0,0.0,t70+t71-t17.*t20.*t39.*t47-t26.*t33.*t39.*t46+t2.*t26.*t27.*t50.*(t30+t31-t36-t37).*1.949505e1+t2.*t26.*t35.*t50.*(t30+t31-t36-t37).*(1.0./2.0),t69,t47.*(t72+t73)+t61.*t69-t58.*(t70+t71-t17.*t20.*t39.*t47-t26.*t33.*t39.*t46+t2.*t26.*t27.*t33.*t50.*1.949505e1+t2.*t26.*t33.*t35.*t50.*(1.0./2.0))+t46.*(t2.*t17.*t20.*t39-t5.*t26.*t33.*t39),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,-t5.*t17.*t39.*t55-t2.*t26.*t39.*t53,t77,t61.*t77+t58.*(t5.*t17.*t39.*t55+t2.*t26.*t39.*t53)],[6, 8]);