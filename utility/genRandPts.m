function [ pts ] = genRandPts(n,dim,range,method)
%GENRANDPTS generates a set of pseudorandom or quasirandom points 
%@param n - number of points
%@param dim - dimension of points 
%@param dim x 2 matrix of max and min range 
%@param method - method for generating 
    %@option rand - default
    %@option randn
    %@option halt

%@output pts - dim x n matrix of points
if strcmp(method,'halt')
    %generate halton points
    p = haltonset(dim,'Skip',1e3,'Leap',1e2);
    p = scramble(p, 'RR2');
    pts = net(p,n)';
elseif strcmp(method,rand)
    pts = rand(dim,n);
elseif strcmp(method,randn)
    pts = rand(dim,n);
else
    pts = rand(dim,n);
end
pts = (range(:,2)-range(:,1))*ones(1,n).*pts...
                +range(:,1)*ones(1,n);


end

