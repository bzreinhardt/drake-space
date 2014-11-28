function genFnFromFit(sfit)
form = formula(sfit);
coeffs = coeffvalues(sfit);
indep = indepnames(sfit);
names = coeffnames(sfit);
fnstr = ''; 
%need to check that protected functions don't get overwritten like sin or
%exp
ops = '*+/-) ';
for i = 1:length(form) -1 
    %check if it's a var followed by an operator
    if (any([names{:}] == form(i)) && ...
            any(form(i+1) == ops))
        fnstr = strcat(fnstr,num2str(coeffs(find([names{:}] == form(i)))));
    elseif (any(form(i) == '*/^'))
        fnstr = strcat(fnstr,'.',form(i));
    else
        fnstr = strcat(fnstr,form(i));
    end      
end
% you wont have a coefficient or an operator at the end of a formula
fnstr = strcat(fnstr,form(length(form)));

%save to a file later
disp(fnstr);
end