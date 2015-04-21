function ws_struct = workspaceToStruct()
%writes entire workspace to a struct
    ws_vars = evalin('caller','who');
    ws_struct = struct();
    for i = 1:length(ws_vars)
        
        ws_struct.(ws_vars{i}) = evalin('caller',ws_vars{i});
    end
end