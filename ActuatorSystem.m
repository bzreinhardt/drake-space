classdef ActuatorSystem < MIMODrakeSystem
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        num_actuators = 0;
        actuator_type; %the type of the actuators so the force elements can be searched
        target_copy; %A stored copy of the system being actuated 
        
    end
    
    methods
        function obj = ActuatorSystem(num_xc,num_xd,inputFrame,outputFrame,direct_feedthrough_flag,time_invariant_flag,num_actuators)
            obj = obj@MIMODrakeSystem(num_xc,num_xd,inputFrame,outputFrame,direct_feedthrough_flag,time_invariant_flag);
        end
        
        function xcdot = mimoDynamics(obj,t,x,varargin)
        end
        
        function varargout = mimoOutput(obj,t,x,varargin)
            for i = 1:obj.num_actuators
                %find the inputs for the actuator and add them to 
            end
        end
    end
    
end

