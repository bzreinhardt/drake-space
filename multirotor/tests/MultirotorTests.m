classdef MultirotorTests
%TESTS for controlling the quad rotor
methods(Static)
function testConnectControlRealDynamics %works 1/1
    sys = MultiRotor;
    x_range = [-0.1 0.1];
    y_range = [0 0];
    theta_range = [-pi pi];
    dx_range = [0 0];
    dy_range = [0 0];
    dtheta_range = [0 0];
    range = [x_range; y_range; theta_range; dx_range; dy_range; dtheta_range];
    x0 = [[0;0;-0.2; zeros(3,1)],[0;0;0; zeros(3,1)]];
    prm = LQRPRM(sys,range);
    options = struct();
    %options.plot = 'static';
    options.method = 'tilqr';
    options.normalize = false;
    

    for i=1:size(x0,2)
        options.x0 = x0(:,i);
        
        prm = prm.genControlRegion(options);

    end
    
    
     prm = prm.findVolume(options);
     disp(prm.volume);
    
     
    fprintf('\n total volume = %f \n',prm.volume*1E10);
    valuecheck(size(prm.occupancy_map),[2,2]);
    disp(prm.occupancy_map);
    figure(1337);clf;
    options.dims = [1,3];
    prm.draw(gcf,options);
 %   prm.save;
end
end


end

