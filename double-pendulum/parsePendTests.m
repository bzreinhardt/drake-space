function [volumes,iterations,baseline] = parsePendTests( folder,plot_opts )
%PARSETESTS parses a the data from a folder and plots the control volumes
% for the automatically generated designs
list = ls(folder);
files = textscan(list,'%s');
filenames = files{1};
volumes = [];
iterations = [];
baseline = struct();
for i = 1:numel(filenames)
    if isempty(strfind(filenames{i},'baseline')) && isempty(strfind(filenames{i},'control'))
    [controller, design,its] = readControllerDesign(strcat(folder,'/',filenames{i}));
    
    volumes(end+1) = controller.volume;
    iterations(end+1) = its.iterations;
    else
        [baseline.controller,baseline.design,baseline.its] = ...
            readControllerDesign(strcat(folder,'/',filenames{i}));
    end
end
if nargin > 1
    figure(1);clf;
    scatter(iterations,volumes);
end
end

