function [Bx, By] = plotHalbachField(x_lim,y_lim,Br,P,ur,r_o,r_i)
%Finds a contour plot of the magnetic field of a halbach rotor centered at
%0 0 in the limits [xmin xmax] [ymin ymax] given

xgv = linspace(x_lim(1),x_lim(2),50);
ygv = linspace(y_lim(1),y_lim(2),50);
[X,Y] = meshgrid(xgv,ygv);
C = findC(Br,P,ur,r_o,r_i);
R = (X.^2+Y.^2).^0.5;
X = X(R>r_o);
Y = Y(R>r_o);
B = C*(X-1i*Y).^-2;
Bx = real(1i*B);
By = real(B);
%Start x around a circle

streamline(X,Y,Bx,By);
end