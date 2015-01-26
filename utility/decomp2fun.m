function fun = decomp2fun(M,p,n,str_output)
%create a function handle for the polynomial represented by the matrix M
%and polynomial p
%iterate over the terms of M
if nargin < 4
    str_output = false;
end
fn = '';
for i = 1:size(M,2) 
    term = sprintf('%i',full(M(n,i)));
    for j = 1:size(p,2)
        if p(i,j) ~= 0
            term = strcat(term,sprintf('*x(%d)^%i',j,full(p(i,j))));
        end
    end
    if i ~= 1 && i ~= size(M,2)
        fn = strcat(fn,'+',term);
    elseif i == size(M,2)
        fn = strcat(fn,'+',term);
    elseif i == 1
        fn = strcat(fn,term);
    end
end
if str_output == false
    fun = str2func(strcat('@(x)',fn));
else
    fun = fn;
end