 function f = fourierForce(B_s, Gamma, w)
            %fourierForce finds the steady state force from a single em source with a
            %single time frequency.

            %INPUTS
            % B_s - A function handle wrt xi that is the constant component of the source field. 
            %fourier transformed in the x direction. Complex valued: B_s = B_x + i*B_y
            % Gamma - a function handle wrt xi that is a propigation function through the material
            % w - length of the problem (the plate) in the z direction
            %OUTPUT
            % f- force 2x1 fx,fz
            mu0 = 4*pi*10^-7; % mu doesn't really change
            endPt = 1E3; % used to avoid problems at infinity with EM forces

            integrand = @(xi)(abs(B_s(xi)).^2.*Gamma(xi));
            im_force = w/(8*pi*mu0)*(integral(integrand,-endPt,endPt));
            %negatives to have force on magnet rather than force on plate
            f = [-imag(im_force);-real(im_force)];

        end