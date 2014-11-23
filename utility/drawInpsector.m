function drawInspector(a,d)
gen = InspectorGenerator();
gen = gen.setCouplers(a,d);
gen.genFile('draw_inspector_out.urdf');
r = InductionInspector('draw_inspector_out.urdf');
v = r.constructVisualizer();


    