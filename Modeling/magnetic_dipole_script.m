%manually entered data from KJ magnetics site
mag_data_in_gauss = mag_data;
%r = sym('r');
%m = sym('m');
mu0 = 1.25663706E10-6; % m kg s-2 A-2
%B = mu0/(4*pi)*(3*m/r^3);
%m = 4/3*pi/mu0*r^3;

% conver the data to SI units
Tesla_per_gauss = 0.0001; %T/G
m_per_inch = 0.0254; %m/in
temp = mag_data_in_gauss;
mag_data_m_T = [temp(:,1:2)*m_per_inch, temp(:,3)*Tesla_per_gauss];
B = mag_data_in_gauss(:,3);
one_over_r_cubed = mag_data_m_T(:,2).^-3;
C = mu0/(4*pi)*2*one_over_r_cubed;
guess_m = pinv(C'*C)*C'*B;

%guess m = 3.507 E-13 