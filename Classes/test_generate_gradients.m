% Test automatic gradient gneration
p = PlanarInspector();
m = size(p.d,1); %number of couplers
generateGradients('dynamics',1,'planarGradients',p,0,randn(6,1),randn(m,1));