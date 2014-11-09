classdef EMModel
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        a2 = 0.41;%Coil outer radius
        a1 = 0.52;%Coil inner ratius
        a = 0.03;%Coil mean radius
        b = 0.16;%Coil axial thickeness
        c = 0.2; %m%Coil radiual thickness
        N = 107; %Coil turns
        L = 980E-6; %Henrys %coil inductance in free space
        R = 0.38; %ohms%coil resistance
        m = 0.35; %kg%coil mass
        L0; %calculated coil inductance
    end
    
    methods
        function obj = EMModel(varargin)
            obj.a = (obj.2+obj.a1)/2;
            L0 = obj.N^2*3.15*obj.a^2/100/(6*obj.a+9*obj.b+10*obj.c);
            %note that inductance as a function of frquency is much harder
        end
    end
    
end

