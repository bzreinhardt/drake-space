function [B] = dipoleFlux( X, Xmag, m )

u0 = 1; 
r = X - Xmag;
B = u0/(4*pi)*(3*r*dot(m,r)/norm(r)^5-m/norm(r)^3);
end