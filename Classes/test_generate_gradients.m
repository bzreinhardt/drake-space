% Test automatic gradient gneration
p = PlanarInspector();

generateGradients('dynamics',1,'autoGenPlanarGradients',p,0,randn(6,1),randn(2,1));