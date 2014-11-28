%%% Fit a polynomial function to the induction coupler forces
% u = linspace(-10000,10000,100);
% g = linspace(0.001,1,50);
% g = g.^3;
% vx = linspace(0,4);
% vy = linspace(0,4);
% 
% [U,G] = meshgrid(u,g);
% 
% Fx = zeros(size(G));
% Fy = zeros(size(G));
% ic = InductionCouplerModel();
% for i = 1:numel(G)
%     
%         [F] = ic.findForce([0;G(i)],[0;0],U(i));
%         Fx(i) = F(1);
%         Fy(i) = F(2);
% 
% end
% fx_u = Fx(1,:);
% fy_u = Fy(1,:);
% fx_g = Fx(:,1);
% fy_g = Fy(:,1);
% 
% if ~exist('fy_ug','var')
%     load('continuous_fits.mat');
% end
% 
% fx_ug_formula = formula(fx_ug);
% fx_ug_coeffs = coeffvalues(fx_ug);
% fx_ug_indep = indepnames(fx_ug);
% fx_ug_names = coeffnames(fx_ug);
% fx_ug_fnstr = '';
% for i = 1:length(fx_ug_formula)
%     if any([fx_ug_names{:}] == fx_ug_formula(i))
%         fx_ug_fnstr = strcat(fx_ug_fnstr,num2str(fx_ug_coeffs(find([fx_ug_names{:}] == fx_ug_formula(i)))));
%     
%     elseif (any(fx_ug_formula(i) == '*/^'))
%         fx_ug_fnstr = strcat(fx_ug_fnstr,'.',fx_ug_formula(i));
%     else
%         fx_ug_fnstr = strcat(fx_ug_fnstr,fx_ug_formula(i));
%     end      
% end
% %save to a file later
% disp(fx_ug_fnstr); 
%generated with genFnFromFit

fx = @(x,y)((-5.6219e-08.*x.^3+6.2003e-06.*x.^2+76.0622.*x).*6.2003e-06.*exp(-44.1463.*y));
fy = @(x,y) ((5.811e-09.*x.^4+-2.2676e-08.*x.^3+-1.8637.*x.^2+0.29795.*x).*-33.8115.*exp(-38.9901.*y+-21.8594));

fx_fit = fx(u,0.01);
fy_fit = fy(u,0.01);
fx_real = zeros(size(u));
fy_real = zeros(size(u));
for i = 1:length(u)
    F = ic.findForce([0;0.01],[0;0],u(i));
    fx_real(i) = F(1);
    fy_real(i) = F(2);
end
figure(1);clf;
subplot(211);
plot(u,fx_fit,u,fx_real);
subplot(212)
plot(u,fy_fit,u,fy_real);

fx_fitg = fx(100,g);
fy_fitg = fy(100,g);
fx_realg = zeros(size(g));
fy_realg = zeros(size(g));
for i = 1:length(g)
    F = ic.findForce([0;g(i)],[0;0],100);
    fx_realg(i) = F(1);
    fy_realg(i) = F(2);
end
figure(2);clf;
subplot(211);
plot(g,fx_fitg,g,fx_realg);
subplot(212)
plot(g,fy_fitg,g,fy_realg);
    