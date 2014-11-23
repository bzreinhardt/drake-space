  function B_s = halbachSource(xi, C, P, r_o, g, x)

            B_s = 4*pi*C*xi.^P/factorial(P).*exp(-xi.*(r_o + g +1i*x)).*(xi >= 0);
            B_s(isnan(B_s))= 0;
  end
         %% CONSTANT FOR HALBACH MAGNET
        function C = findC(Br,P,ur,r_o,r_i)
            %find constant C for a halbach rotor
            %variables
            %Br - magnet remittance
            %P
            %ur - relative permeability
            %r_o - outer radius
            % r_i - inner radius
            
            C = -(2*Br*P)/(P+1)*((1+ur)*r_o^(2*P)*(r_o^(P+1)-r_i^(P+1)))/...
                ((1-ur)^2*r_i^(2*P)-(1+ur)^2*r_o^(2*P));
        end
        