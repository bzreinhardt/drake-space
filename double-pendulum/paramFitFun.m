%%%% Set up fitness functions %%%%%%
    function x = paramFitFun(params,p)
    % p - struct with options
    % p.pend_opts - struct with options for pendulum fit functions
        pend = DoublePendPlant(params(1:2),params(3:4));
        x = pendFitFun(pend,p.pend_opts);
        %fprintf('fit fun eval');
    end

    function [fitness] = pendFitFun(pend,opts)
        %finds the fitness for a given pendulum 
        prm = LQRPRM(pend,opts.range);
        prm.regions_max = opts.max_regions;
        options = struct;
        options.method = 'tilqr';
        prm = prm.fillRegion(options);
        fitness = -prm.volume;
        %disp('volume = ');
        %disp(fitness);
    end