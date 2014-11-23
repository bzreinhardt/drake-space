function [r,v] = drawInspector(a,d)
gen = InspectorGenerator();
gen = gen.setCouplers(a,d);
path_to_inductionInspector = '/home/ben/drake-space/@InductionInspector';
file_name = '/models/draw_inspector_out.urdf';
gen.genFile(strcat(path_to_inductionInspector,file_name));
r = InductionInspector(file_name);
v = r.constructVisualizer();


    