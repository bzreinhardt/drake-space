function forceCallback(q,model,f_x_spline,f_y_spline)
%Finds the actuation possibility in 3 space


d1 = 0.5*[1/2^.5,1/2^.5,-0.1];
d2 = 0.5*[-1/2^.5,1/2^.5,-0.1];
d3 = 0.5*[1/2^.5,-1/2^.5,-0.1];
a1 = d1/norm(d1);
a2 = d2/norm(d2);
a3 = d3/norm(d3);

a = [a1;a2;a3];
d = [d1;d2;d3];

X = [q;zeros(length(q),1)];
[x,v,a_world] = testKinematics(X,a,d);
r = 5; c = [0;0;-r];
[g, surf_norm] = sphereNormGap(x, r, c);
actuator = @(u)findNetForce(u,a_world,g,v,surf_norm,f_x_spline,f_y_spline);
plotActuator(actuator,3);
hold on;
xlabel('f_x');ylabel('f_y');zlabel('f_z');
str = strcat(strcat('x = ',num2str(q(1))),...
    strcat(' y = ',num2str(q(2))),...
strcat(' z = ',num2str(q(3))),...
strcat(' r = ',num2str(q(4))),...
strcat(' p = ',num2str(q(5))),...
strcat(' y = ',num2str(q(6))));
title(str)

drawnow;
hold off;
end

