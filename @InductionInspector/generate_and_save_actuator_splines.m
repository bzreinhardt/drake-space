%Test the optimization of forces and torques for a three armed guy

%create an inductorcouplermodel to find the splines

c = InductionCouplerModel();
% Set up properties of the coupler
divs = struct();
divs.g = 15;
divs.v_x = 10;
divs.v_y = 10;
divs.w = 6;

[f_x_spline, f_y_spline] = c.findSplines( divs);

save('coupler_splines.mat',f_x_spline, f_y,spline);

