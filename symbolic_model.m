syms v_y mu_0 sigma w_e v_x xi b

lambda = v_y * mu_0 * sigma/2;

s_0 = 1i*(w_e + xi*v_x);

gamma = sqrt(xi^2 + mu_0 * sigma * s_0);

beta = sqrt(lambda^2 + gamma^2);

Gamma = (mu_0*sigma*(s_0 - v_y * xi) )/...
    (2 * xi^2 + mu_0 * sigma * s_0 + 2*beta*xi*coth(beta*b));

dGamma_dw = diff(Gamma,w_e);
% dGamma_dw =
 
%(mu_0*sigma*i)/(2*xi^2 + 2*xi*coth(b*(xi^2 + mu_0*sigma*(w_e*i + v_x*xi*i) + (mu_0^2*sigma^2*v_y^2)/4)^(1/2))*(xi^2 + mu_0*sigma*(w_e*i + v_x*xi*i) + (mu_0^2*sigma^2*v_y^2)/4)^(1/2) + mu_0*sigma*(w_e*i + v_x*xi*i)) - (mu_0*sigma*(mu_0*sigma*i - b*mu_0*sigma*xi*(coth(b*(xi^2 + mu_0*sigma*(w_e*i + v_x*xi*i) + (mu_0^2*sigma^2*v_y^2)/4)^(1/2))^2 - 1)*i + (mu_0*sigma*xi*coth(b*(xi^2 + mu_0*sigma*(w_e*i + v_x*xi*i) + (mu_0^2*sigma^2*v_y^2)/4)^(1/2))*i)/(xi^2 + mu_0*sigma*(w_e*i + v_x*xi*i) + (mu_0^2*sigma^2*v_y^2)/4)^(1/2))*(w_e*i + v_x*xi*i - v_y*xi))/(2*xi^2 + 2*xi*coth(b*(xi^2 + mu_0*sigma*(w_e*i + v_x*xi*i) + (mu_0^2*sigma^2*v_y^2)/4)^(1/2))*(xi^2 + mu_0*sigma*(w_e*i + v_x*xi*i) + (mu_0^2*sigma^2*v_y^2)/4)^(1/2) + mu_0*sigma*(w_e*i + v_x*xi*i))^2

dGamma = matlabFunction(dGamma_dw,'file','dGammadW');

