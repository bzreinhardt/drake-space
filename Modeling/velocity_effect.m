%% dependency exploration %%
%load('coupler_splines.mat')
u = linspace(-10000,10000,1000);
g = linspace(0.001,1.5);
vx = linspace(0,4);
vy = linspace(0,4);

% %plot the effect of an x velocity
% fx = fnval(f_x_spline,[vx;zeros(size(vx));0.01*ones(size(vx));zeros(size(vx))]);
% fy = fnval(f_y_spline,[vx;zeros(size(vx));0.01*ones(size(vx));zeros(size(vx))]);
% f_max = fnval(f_x_spline,[0;0;0.1;5000]);
% figure(1);
% plot(vx,fx,vx,f_max*ones(size(vx)));
% figure(2)
% plot(vx,fy,vx,f_max*ones(size(vx)));

%okay now lets see if we can reasonably fit functions to the forces

fx = fnval(f_x_spline, [zeros(size(u));zeros(size(u)); 0.01*ones(size(u));u]);
fy = fnval(f_y_spline, [zeros(size(u));zeros(size(u)); 0.01*ones(size(u));u]);
figure(3);
subplot(211);
plot(u,fx);
subplot(212);
plot(u,fy);

fx_gap = [fnval(f_x_spline, [zeros(size(u));zeros(size(u)); 0.01*ones(size(u));u]);...
    fnval(f_x_spline, [zeros(size(u));zeros(size(u)); 0.05*ones(size(u));u]);
    fnval(f_x_spline, [zeros(size(u));zeros(size(u)); 0.1*ones(size(u));u]);
    fnval(f_x_spline, [zeros(size(u));zeros(size(u)); 1*ones(size(u));u])];
figure(4);
plot(u,fx_gap);

gap_dependency = fnval(f_x_spline, [zeros(size(g));zeros(size(g)); g;1000*ones(size(g))]);
normed_gap_dependency = gap_dependency/(fnval(f_x_spline, [0;0;1;1000]));
figure(5);
plot(normed_gap_dependency); hold on;
plot(1./(g).^3);
hold off;

    


