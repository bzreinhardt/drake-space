%% Generate drawing showing that for low speeds Fx>>Fy for halbach array
save_tikz_on = 0;
save_svg_on = 1;
%%induction couplers
load('coupler_splines.mat');
params.spline.x = f_x_spline;
params.spline.y = f_y_spline;
u = -300:1:300;
g = 0.01*ones(size(u));
vx = zeros(size(u));
vy = zeros(size(u));
fx = fnval(f_x_spline,[vx;vy;g;u]);
fy = fnval(f_y_spline,[vx;vy;g;u]);
figure(1);
clf;
subplot(211)
h = plot(u,fx,'--g','LineWidth',2);
ylabel('Force (N)');
title('Tangential Force');
subplot(212);
h2 = plot(u,fy*100,'g','LineWidth',2);
title('Normal Force');
xlabel('Array Speed (rad/s)');
ylabel('Force (N*10E-2)');
if save_tikz_on
 matlab2tikz('/home/ben/Documents/papers/two_d_inspection_paper/figures/tan_v_norm_force.tikz','height', '\figureheight', 'width', '\figurewidth');
end
if save_svg_on
    plot2svg('/home/ben/Documents/papers/thesis/figures/svgs/tan_v_norm_force.svg');
end
%% Demonstrate linearity with omega at low speeds

inspection_sim_setup
fx = zeros(size(u));
for i = 1:length(u)
    array.w_e = -u(i);
    F = array.genForce();
    fx(i) = F(2);
end


[fit] = polyfit(u,fx,1);
yfit = polyval(fit,u);
SSresid = sum((fx - yfit).^2);
SStotal = (length(fx)-1)*var(fx);
rsq = 1 - SSresid/SStotal;

figure(2); clf;
h2 = plot(u,fx,'--','LineWidth',5);
hold on
plot(u,fit(1)*u+fit(2),'LineWidth',3);
set(gca,'FontSize',16);
legend('Full Nonlinear Model','Linear Fit');
xlabel('Array Speed (rad/s)');
ylabel('Force (N)');
if save_on
matlab2tikz('/home/ben/Documents/papers/two_d_inspection_paper/figures/lin_fit.tikz','height', '\figureheight', 'width', '\figurewidth');
end