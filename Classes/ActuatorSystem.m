classdef ActuatorSystem < MIMODrakeSystem
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        num_actuators = 0;
        actuator_type; %the type of the actuators so the force elements can be searched
        target_copy; %A stored copy of the system being actuated
        actuators;
        actuator_shadows;
        
    end
    
    methods
        function obj = ActuatorSystem(num_xc,num_xd,inputFrame,outputFrame,direct_feedthrough_flag,time_invariant_flag,num_actuators)
            obj = obj@MIMODrakeSystem(num_xc,num_xd,inputFrame,outputFrame,direct_feedthrough_flag,time_invariant_flag);
            obj.num_actuators = num_actuators;
        end
        
        function xcdot = mimoDynamics(obj,t,x,varargin)
        end
        
        function varargout = mimoOutput(obj,t,x,varargin)
            u = varargin{1}; %actual control input
            q = varargin{2}; %body pos vars
            %qd = varargin{3}; %body vel vars 
            
            f_out = zeros(obj.num_actuators,1);
            for i = 1:obj.num_actuators
                %find the input values for the actuator and add them to the output 
                f_out(i) = norm(u)+norm(q);
            end
            varargout = {f_out};
        end
        function obj = setTarget(obj,target)
            %keeps a deep copy of the system being actuated for force-finding
            %@param - a copy of the target system
            obj.target_copy = target;
            %cycle through the force objects on the target and assign each
            %one a shadow
            for i = 1:obj.num_actuators
            end
        end
    end
    
end

