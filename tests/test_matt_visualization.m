% MAIN - Demo - Nonlinear Basis Functions
%
% Suppose that you have the following function:
%
% P = f1(u1) + f2(u2) 
%
% Where P, f1, and f2 are in R3 and u1 and u2 are in R1
%
% Given bounds on u1 and u2, and the functions f1 and f2, what does P look
% like?
%

n = 100;
u1 = linspace(-1,1,n);
u2 = linspace(-1,1,n);

f1 = [atan(4*u1);u1.^2;0*u1];
f2 = [0*u2;atan(4*u2);u2.^2];

%Plot basis functions (f1 and f2)
figure(1337); clf; hold on;
plot3(f1(1,:),f1(2,:),f1(3,:),'b-','LineWidth',4)
plot3(f2(1,:),f2(2,:),f2(3,:),'r-','LineWidth',4)
view(45,45);

%Plot Level curves of P
z = zeros(3,n*n);
ONES = ones(1,n);
for i=1:n
    idx = (i-1)*n + (1:n);
   z(:,idx) = f1(:,i)*ONES + f2; 
   plot3(z(1,idx),z(2,idx),z(3,idx),'k-','LineWidth',2)
end