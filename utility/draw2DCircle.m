function draw2DCircle(center,rad,varargin)
%draws a circular patch object in the xy plane
%@param center
%@param rad
%@param varargin - additional arguments a patch could take

theta = linspace(0,2*pi,50);
patch(center(1)+rad*cos(theta),center(2)+rad*sin(theta),0*theta,varargin{:});
end