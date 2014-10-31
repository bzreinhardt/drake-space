function [bcs_x, bcs_y] = inductionCoupler_test

%% create a grid of state and input values
v_x = [-10:1:10];
v_y = [-10:1:10];
x = [-1:0.1:1];
y = [0.001:0.1:1];
phi = [0:0.1:pi];
g = y.^3;
w = [-10000:500:10000];
%% Test multidim spline
% testF = zeros(length(v_x), length(v_y), length(g));
% for i = 1:length(v_x)
%     for j = 1:length(v_y)
%         for k = 1:length(g)
%             
%             testF(i,j,k) = v_x(i)+v_y(j)+g(k);
%         end
%     end
% end
%% Check Smoothness
% c = InductionCouplerModel();
% Fx = zeros(length(w),length(g));
% Fy = zeros(length(w),length(g));
% for i = 1:length(w)
%     for j = 1:length(g)
%         F = c.findForce([0,g(j)],[0 0],w(i));    
%         Fx(i,j) = F(1);
%         Fy(i,j) = F(2);
%         
%     end
% end
% figure(1);clf; subplot(211);plot(repmat(w', [1,length(g)]),Fx);
% subplot(212);plot(repmat(w', [1,length(g)]),Fy);

% legend(cellstr(num2str(g','g=%f')));
%% Find cubic splines over reasonable workspace
c = InductionCouplerModel();
Fx = zeros(length(v_x), length(v_y), length(g));
Fy = zeros(length(v_x), length(v_y), length(g));
disp('Finding forces over workspace ...');
for i = 1:length(v_x)
    disp(strcat('i = ',num2str(i)));
    for j = 1:length(v_y)
        for k = 1:length(g)
            
                F = c.findForceDeriv([0,g(k)],[v_x(i) v_x(j)],0); 
                Fx(i,j,k) = F(1);
                Fy(i,j,k) = F(2);
            
        end
    end
end
disp('Solving for splines')
bcs_x = csapi( {v_x,v_y,g}, Fx );
bcs_y = csapi( {v_x,v_y,g}, Fy );

nonlinearF = zeros(length(w),2);
linearF = zeros(length(w),2);
for i = 1:length(w)
    nonlinearF(i,:) = (c.findForce([0,0.01],[0 0],w(i)))';
    linearF(i,:) = [fnval(bcs_x,{0,0,0.1})*w(i),fnval(bcs_y,{0,0,0.1})*w(i)];
end

figure(1);clf; subplot(211);plot(w,nonlinearF(:,1),w,linearF(:,1));
subplot(212);plot(w,nonlinearF(:,2),w,linearF(:,2));
% subplot(212);plot(repmat(w', [1,length(g)]),Fy);

