%design control inspector defaults
function saveDefaultsAsJson(filename)
    if nargin < 1
        filename = 'inspector_default_values.json';
    end
    x_range = [-0.05 0.05];
    y_range = [0.11 0.15];
    theta_range = [0 0];
    dx_range = [0 0];
    dy_range = [0 0];
    dtheta_range = [0 0];
    range = [x_range; y_range; theta_range; dx_range; dy_range; dtheta_range];
    max_regions = 20;
    defaults = workspaceToStruct();
    savejson('',defaults,filename);
end