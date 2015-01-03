function addpath_drakespace()
root = pwd;
%add useful folders here
addpath(fullfile(root)); % add everything for now
addpath(fullfile(root,'utility'));
addpath(fullfile(root,'planar-inspector'));
addpath(fullfile(root, 'control-metric'));
