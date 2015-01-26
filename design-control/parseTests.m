function parseTests( folder )
%PARSETESTS parses a the data from a folder and plots the control volumes
% for the automatically generated designs
list = ls(folder);
files = textscan(list,'%s');
filenames = files{1};
for i = 1:numel(filenames)
    [controller, design] = readControllerDesign(filenames{i});
    arms(i) = size(design.a,2);
    volumes(i) = controller.volume;
end

figure(1);clf;
scatter(arms,volumes);

end

