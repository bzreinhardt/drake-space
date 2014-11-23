n = 25;
f_max = 2;
f_min = -2;
fx = linspace(f_min,f_max,n);
fy = linspace(f_min,f_max,n);
fz = linspace(f_min,f_max,n);
[Fx,Fy,Fz] = ndgrid(fx,fy,fz);
f = [reshape(Fx,1,n*n*n); reshape(Fy,1,n*n*n);reshape(Fz,1,n*n*n)];
