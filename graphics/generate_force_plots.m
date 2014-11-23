%% Generate drawing showing that for low speeds Fx>>Fy for halbach array
%%induction couplers
load('coupler_splines.mat');
params.spline.x = f_x_spline;
params.spline.y = f_y_spline;
u = 0:1:300;
g = 0.01*ones(size(u));
vx = zeros(size(u));
vy = zeros(size(u));
fx = fnval(f_x_spline,[vx;vy;g;u]);
fy = fnval(f_y_spline,[vx;vy;g;u]);
figure(1);
clf;
h = plot(u,fx,'--',u,fy,'LineWidth',2);
set(gca,'FontSize',16);
legend('Tangential Force','Normal Force');
xlabel('Array Speed (rad/s)');
ylabel('Force (N)');
ylim([-0.01,0.2]);
save2pdf('/home/ben/Documents/papers/two_d_inspection_paper/figures/tan_v_norm_force.pdf');
%% Demonstrate linearity with omega at low speeds

fit = polyfit(u,fx,1);

figure(2);
h2 = plot(u,fx,u(1:10:end),fit(1)*u(1:10:end)+fit(2),'o','LineWidth',2);
set(gca,'FontSize',16);
legend('Modeled Force','Linear Fit');
xlabel('Array Speed (rad/s)');
ylabel('Force (N)');
save2pdf('/home/ben/Documents/papers/two_d_inspection_paper/figures/lin_fit.pdf');