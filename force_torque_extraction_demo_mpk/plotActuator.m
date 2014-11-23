function plotActuator(actuator,num_u)

n = 50;
u_min = -1E4;
u_max = -u_min;
u = repmat(linspace(u_min,u_max,n),num_u,1);
u_cell = num2cell(u,2);
u_cell = u_cell';

ui = blkdiag(u_cell{:});

 f = actuator(ui);
 
%Plot basis functions (f1 and f2)
cc = hsv(12);
figure(1337);
for i = 0:num_u-1
plot3(f(1,n*i+1:n*i+n),f(2,n*i+1:n*i+n),f(3,n*i+1:n*i+n),'-','Color',cc(i+6,:),'LineWidth',4) ;
hold on;
end

%plotActuationSpan(actuator,num_u);

%Plot Level curves of P
% z = zeros(3,n*n);
% ONES = ones(1,n);
% 
% for i=1:n
%     idx = (i-1)*n + (1:n);
%    z(:,idx) = f1(:,i)*ONES + f2; 
%    plot3(z(1,idx),z(2,idx),z(3,idx),'k-','LineWidth',2)
% end
hold off
end