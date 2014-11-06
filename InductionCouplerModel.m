classdef InductionCouplerModel 
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
   
    
    methods
         function obj = InductionCoupler(options)
             obj.C = obj.findC();
         end
        %% SET PROPERTIES
        function obj = set(obj,names, vals)
            if length(names) ~= length(vals)
                error('Names and vals must be the same length')
            end
            for i = 1:length(names)
                if isfield(obj,names{i})
                    obj.(matlab.lang.makevalidname(names{i})) = vals(i);
                else
                    error('Not a field');
                end
            end
            obj = findC(obj);
        end
       
        
 
    
         %% FIND CONSTANT FOR HALBACH MAGNET
        function obj = findC(obj)
            %find constant C for a halbach rotor
            %variables
            %Br - magnet remittance
            %P
            %ur - relative permeability
            %r_o - outer radius
            % r_i - inner radius
            
            obj.C = -(2*obj.Br*obj.P)/(obj.P+1)*((1+obj.ur)*obj.r_o^(2*obj.P)*(obj.r_o^(obj.P+1)-obj.r_i^(obj.P+1)))/...
                ((1-obj.ur)^2*obj.r_i^(2*obj.P)-(1+obj.ur)^2*obj.r_o^(2*obj.P));
        end
        %% FIND THE FORCE FROM THE COUPLER
        function f = findForce(obj,g,x,xdot,w_e)
            %fourierForce finds the steady state force from a single em source with a
            %single time frequency.
            
            
            %OUTPUT
            % f- force 2x1 fx,fz
            if strcmp(obj.type,'pm')
                B_s = @(xi)obj.halbachSource(xi,obj.C,obj.P,obj.r_o, g,0);
                Gamma = @(xi)obj.findGamma(xi,xdot(1),xdot(2),obj.mu0, obj.sigma, w_e,obj.b);
                endPt = 1E3; % used to avoid problems at infinity with EM forces
                integrand = @(xi)(abs(B_s(xi)).^2.*Gamma(xi));
                im_force = obj.w/(8*pi*obj.mu0)*(integral(integrand,-endPt,endPt));
                %negatives to have force on magnet rather than force on plate
                f = [-imag(im_force);-real(im_force)];
            elseif strcmp(obj.type,'em')
                %Need to put em force model here
            end
            
        end
        
        
        
    end
    
    methods(Static)
           function B_s = halbachSource(xi, C, P, r_o, g, x)
            
            B_s = 4*pi*C*xi.^P/factorial(P).*exp(-xi.*(r_o + g +1i*x)).*(xi >= 0);
            B_s(isnan(B_s))= 0;
           end
        
            function Gamma = findGamma(xi, v_x, v_y, mu0, sigma, w_e,b)
            %finds the big gamma transmission propigation function from the bird style
            %steady-state force solution to eddy current propigation
            %INPUTS
            %OUTPUT
            
            s0 = 1i*(w_e*ones(size(xi)) - xi*v_x);
            lambda = v_y * mu0*sigma/2*ones(size(xi));
            gamma2 = xi.^2+mu0*sigma*s0.*ones(size(xi));
            beta2 = lambda.^2+gamma2;
            beta = sqrt(beta2);
            
            Tss_top = ((lambda-(xi+beta)).*exp(beta*b)-(lambda - (xi - beta)).*exp(-beta*b));
            Tss_bottom = exp(beta*b).*(lambda.^2-(xi + beta).^2)-exp(-beta*b).*(lambda.^2-(xi-beta).^2);
            Tss = Tss_top./Tss_bottom;
            
            Gamma = 2.*xi.*Tss - 1;
            
        end
    end
         properties
             C = 0;
        %MAGNET PROPERTIES
        m = 2E-3; %Am^2
        r_o = 2.54*0.75*0.01; %0.75 inches converted to m
        r_i = 2.54*0.5*0.01;  %0.5 inches converted to m
        Br = 1.42; %T Magnet strength
        ur = 1.08; %unitless magnet relative permeability
        P = 1; %pole-pairs
        b = 0.01; %width of the halbach array
        mu0 = 1.256E-6; %m*kg/s^2 A^2 magnetic permeability in a vacuum
        type = 'pm';%indicate whether an electromagnet or permanent magnet
        %PLATE PROPERTIES
        rho = 2.83E-6; %ohm-m resistivity of the plate
        sigma = 1/rho; %conductivity of the plate 
        w = 0.1; %width of the plate
        
        %STATE PROPERTIES
        v_x = 0; %velocity parallel to the plate
        v_y = 0; %velocity perpendicular to the plate
        g = 0.01; %air gap m
        w_e = 30; %electric frequency
    end
        
    
end

