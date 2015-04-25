function [exists] = checkPropertyExists(target, name)
%CHECKPROPERTYEXISTS return 1 if target has a property name
possible_fields = properties(target);
exists = any(strncmp(possible_fields,name,length(name)));

end

