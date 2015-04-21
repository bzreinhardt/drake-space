function baseline_volume = generateBaselineVolume(params)
if nargin < 1
    params = [0.5;0.5;0.5;0.5];
end
%find the volume 
baseline_volume = -pendulumFitness(params);
        

end