function u = stupidFindFixedPoint(sys,x0)
    
        objective = @(u)max(abs(sys.dynamics(0,x0,u)));
        u_upp = sys.umax;
        u_low = sys.umin;
        A = [eye(numel(u_upp));-eye(numel(u_low))];
        B = [u_upp;-u_low];
        options = optimset('TolX', 1e-6);
        u = fmincon(objective,zeros(size(u_upp)),A,B,[],[],[],[],[],options);
    end