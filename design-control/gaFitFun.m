    function fitness = gaFitFun(params,fitFun)
        a = [zeros(2,4);ones(1,4)];
        d = zeros(3,4);
        for i = 1:4
        d(:,i) = 0.1*[cos(params(i));sin(params(i));0];
        end
        insp = Inspector2d(a,d);
        if nargin < 2
            fitness = -fakeFitness(insp);
        else
           fitness =  fitFun(insp);
        end
    end