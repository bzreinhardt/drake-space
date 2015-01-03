function [a,d] = generateCouplerParams(n,dims,options)
%GENERATECOUPLERPARAMS 
%@params - n : number of couplers
%@params - dims: dimension of the problem
%defaults
if nargin < 1
    n = 4; 
end
if nargin < 2
    dims = 3;
end
if nargin < 3
    options = struct();
end
%TODO input parser
if ~isfield(options,'quat_range')
end
if ~isfield(options,'len_range')
end
if ~isfield(options,'axis_range')
end

if dims == 3
    %3 dimensionsal case (xyt)
    %all the axes point in positive z direction
    a = [zeros(2,n); ones(1,n)];
    d = [zeros(3, n)];
    %each coupler strut has a length and an angle 
    options = struct();
    options.max = [0.1;0]*ones(1,n);
    options.min = [0.1;-pi]*ones(1,n);
    
    params = generateParams(d(1:2,:),options); %find angles and lengths
    %STOPPING POINT - write a test
    for i = 1:n
        
        theta = params(2,i); l = params(1,i);
        d(:,i) = [l*cos(theta);l*sin(theta);0];
       
    end
    
    
    
    
elseif dims == 6
end
    
end
