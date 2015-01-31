function [ m,l ] = generatePendParams(mass_range,length_range)
%GENERATEPENDPARAMS generate mass and length parameters for a double
%pendulum
if nargin < 2
    mass_range = [0.5 1.5];
    length_range = [0.5 1.5];
   
end

options.min = [mass_range(1)*ones(2,1);length_range(1)*ones(2,1)];
options.max = [mass_range(2)*ones(2,1);length_range(2)*ones(2,1)];
params = generateParams(zeros(4,1),options);
m = params(1:2);
l = params(3:4);

end

