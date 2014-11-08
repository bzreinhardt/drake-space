classdef InductionCouplerModel 
    %UNTITLED4 Summary of this class goes here
    %   Detailed explanation goes here
    
   
    
    methods
         function obj = InductionCouplerModel(options)
             obj = obj.findC();
             obj.sigma = 1/obj.rho; %conductivity of the plate 
            
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
        
        function obj = setSplines(obj, divs)
            [f_x_spline, f_y_spline] = findSplines(obj, divs);
            obj.f_x = f_x_spline; 
            obj.f_y = f_y_spline;
        end
               %% FIND THE FORCE FROM THE COUPLER
        function f = findForce(obj,x,xdot,w_e)
            %fourierForce finds the steady state force from a single em source with a
            %single time frequency.
            
            
            %OUTPUT
            % f- force 2x1 fx,fz
            if strcmp(obj.type,'pm')
                g = x(2);
                B_s = @(xi)obj.halbachSource(xi,obj.C,obj.P,obj.r_o, g,0);
                Gamma = @(xi)obj.findGamma(xi,xdot(1),xdot(2),obj.mu_0, obj.sigma, w_e,obj.b);
                endPt = 1E3; % used to avoid problems at infinity with EM forces
                integrand = @(xi)(abs(B_s(xi)).^2.*Gamma(xi));
                im_force = obj.w/(8*pi*obj.mu_0)*(integral(integrand,-endPt,endPt));
                %negatives to have force on magnet rather than force on plate
                f = [-imag(im_force);-real(im_force)];
            elseif strcmp(obj.type,'em')
                %Need to put em force model here
                %from Thomson 2000 Electrodynamic Magnetic Suspension...
                
            end
            
        end
        
        %% FIND THE DERIVATIVE OF THE FORCE WRT W
        function df = findForceDeriv(obj,x,xdot,w_0)
            v_x = xdot(1);
            v_y = xdot(2);
            g = x(2);
            w_e = w_0;
            endPt = 1E3; % used to avoid problems at infinity with EM forces
                
            B_s = @(xi)obj.halbachSource(xi,obj.C,obj.P,obj.r_o, g,0);
            dGamma = @(xi)dGammadW(obj.b,obj.mu_0,obj.sigma,v_x,v_y,w_e,xi);
            integrand = @(xi)(abs(B_s(xi)).^2.*dGamma(xi));
             im_dforce = obj.w/(8*pi*obj.mu_0)*(integral(integrand,-endPt,endPt));
              df = [imag(im_dforce);real(im_dforce)];
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
        
        %% FIND INTERPOLATION SPLINES
        function [f_x_spline, f_y_spline] = findSplines(obj, divs)
            
            v_x_min = obj.v_lims(1,1);
            v_x_max = obj.v_lims(1,2);
            v_y_min = obj.v_lims(2,1);
            v_y_max = obj.v_lims(2,2);
            g_min = obj.g_lims(1);
            g_max = obj.g_lims(2);
            w_min = obj.w_lims(1);
            w_max = obj.w_lims(2);
            
            v_x = [v_x_min:(v_x_max-v_x_min)/divs.v_x:v_x_max];
            v_y = [v_y_min:(v_y_max-v_y_min)/divs.v_y:v_y_max];
            g = [g_min:(g_max-g_min)/divs.g:g_max];
            w = [w_min:(w_max-w_min)/divs.w:w_max];
            
            
            Fx = zeros(length(v_x), length(v_y), length(g), length(w));
            Fy = zeros(length(v_x), length(v_y), length(g), length(w));
            disp('Finding forces over workspace ...');
            for i = 1:length(v_x)
                disp(strcat('Progress: v_x = ', num2str(v_x(i)), ' out of ', num2str(v_x(end))));
                for j = 1:length(v_y)
                    for k = 1:length(g)
                        for l = 1:length(w)
                            F = obj.findForce([0,g(k)^2],[v_x(i) v_x(j)],w(l));
                            Fx(i,j,k,l) = F(1);
                            Fy(i,j,k,l) = F(2);
                        end
                    end
                end
            end
            disp('Solving for splines')
            f_x_spline = csapi( {v_x,v_y,g,w}, Fx );
            f_y_spline = csapi( {v_x,v_y,g,w}, Fy );
        end
        
        %% Find power output
        function  P = findPower(obj,F,w)
            P = F*w*obj.r_o;
        end
        
    end
    
    methods(Static)
           function B_s = halbachSource(xi, C, P, r_o, g, x)
            
            B_s = 4*pi*C*xi.^P/factorial(P).*exp(-xi.*(r_o + g +1i*x)).*(xi >= 0);
            B_s(isnan(B_s))= 0;
           end
        
            function Gamma = findGamma(xi, v_x, v_y, mu_0, sigma, w_e,b)
            %finds the big gamma transmission propigation function from the bird style
            %steady-state force solution to eddy current propigation
            %INPUTS
            %OUTPUT
            
            s0 = 1i*(w_e*ones(size(xi)) - xi*v_x);
            lambda = v_y * mu_0*sigma/2*ones(size(xi));
            gamma2 = xi.^2+mu_0*sigma*s0.*ones(size(xi));
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
         w = 0.01; %width of the magnet
        mu_0 = 1.256E-6; %m*kg/s^2 A^2 magnetic permeability in a vacuum
        type = 'pm';%indicate whether an electromagnet or permanent magnet
        eta = 1; %efficiency of the motor powering the magnet
        %PLATE PROPERTIES
        rho = 2.83E-6; %ohm-m resistivity of the plate
        sigma; %conductivity of the plate 
       
        b = 0.005; %plate thickness
        %STATE PROPERTIES
        x_lims = [-10 10; 0.001 2; -pi pi];
         v_lims = [-10 10; -10 10]; %x velocity parallel to the plate
                     %y velocity perpendicular to the plate
         g_lims = [0.001 2]; %air gap m
         w_lims = [-10000 10000]; %electric frequency
         %Calculated Properties
         f_x; 
         f_y;
         
         
    end
        
    
end

