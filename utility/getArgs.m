  function args = getArgs(call)
        i = 1;
        j = 1;
        k = 1;
        args = {};
        % get all the args
        while call(j) ~= ')' && call(i) ~= ')'
            if call(i) == '(' || call(i) == ','
                j = i + 1;
                while call(j) ~= ',' && call(j) ~= ')'
                    j = j + 1;
                end
                args{k} = call(i+1:j-1);
                k = k + 1;
                i = j;
            else
                i = i + 1;
            end
        end
        for k = 1:length(args)
            args{k} = stripSpaces(args{k}); 
        end
    end