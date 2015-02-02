function testSave
t1_range = [0 2*pi];
t2_range = [0 2*pi];

dt1_range = [0 0];
dt2_range = [0 0];

range = [t1_range; t2_range; dt1_range; dt2_range];
options = struct;
options.method = 'tilqr';
baseline_design = DoublePendPlant([0.5;0.5],[0.5;0.5]);
baseline_prm = LQRPRM(baseline_design,range);
baseline_prm = baseline_prm.fillRegion(options);
baseline_prm = baseline_prm.findVolume;
baseline_fitness = baseline_prm.volume;

iterations = 2;
 
date_string = datestr(now);
    date_string((ismember(date_string,' ') == 1)) = '-';
    date_string((ismember(date_string,':') == 1)) = '_';
    filename = strcat('/home/ben/drake-space/data/pendulum_convergence_test/'...
        ,date_string,'.json');
    notes.iterations = iterations;
    


baseline_prm.save(filename,notes);

end