function outstr = stripSpaces(instr)
        outstr = '';
        for i = 1:length(instr)
            if instr(i)~=' '
                outstr = [outstr instr(i)];
            end
        end
            
    end