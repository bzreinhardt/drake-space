function [controller, design,notes] = readControllerDesign( filename)
%READLQRPRM reconstructs a controller and design from a file 

%default file - data from data/arm_num_test
data = loadjson(filename);
design = data.sys_data;

controller.volume = data.volume;
controller.regions = data.controller_data;
controller.occupancy_map = data.map;


if isfield(data,'notes')
    disp(data.notes);
    notes = data.notes;
else
    notes = [];
end
end

