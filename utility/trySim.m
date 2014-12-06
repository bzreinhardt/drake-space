function output = trySim(sim_fn)
success = 0;
tries = 0;
max_tries = 10;

while(success == 0 && tries < max_tries)
    output = feval(sim_fn);
%parse GPOPS outputs
if isfield(output,'result')
    if isfield(output.result,'nlpinfo')
        if output.result.nlpinfo == 1
            success = 1;
        else
            disp(strcat('try failed. exit code ', num2str(output.result.nlpinfo)));
        end
    end
end
end
end