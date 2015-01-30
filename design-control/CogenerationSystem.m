classdef CogenerationSystem
    %abstract class for design control co-generation systems
    methods(Abstract)
        control_region = genControlRegion(obj,x0);
    end
end