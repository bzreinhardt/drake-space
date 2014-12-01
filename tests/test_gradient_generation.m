%%--------------------------%%
%   Test generateGradients  %
% --------------------------%%

cd ~/drake-space/Classes
p = PlanarInspector() ;
generateGradients('noSplineDynamics',1,'dynamicsGradients',p,0,randn(6,1),randn(2,1));