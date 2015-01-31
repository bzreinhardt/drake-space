function testConnectControlRealDynamics %works 1/1
    sys = DoublePendPlant;
    theta1_range = [-pi,pi];
    theta2_range = [-pi,pi];
    omega1_range = [0 0];
    omega2_range = [0 0];
    range = [theta1_range; theta2_range; omega1_range; omega2_range];
    x0 = [[0;0;0;0],[0.05;0;0;0],[0.8*pi;0;0;0]];
    prm = LQRPRM(sys,range);
    options = struct();
    %options.plot = 'static';
    options.method = 'tilqr';
    options.normalize = false;
    

    figure(1337);clf;
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
    prm.draw;
 %   prm.save;
end

