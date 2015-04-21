function saveHumanBaseline
% default human baseline values
angles = [-3*pi/4; -2*pi/3; -pi/3; -pi/4];
[a,d] = armAnglesToInspectorParams(angles);
baseline = workspaceToStruct();
savejson('',baseline,'human_baseline_vals.json');
end