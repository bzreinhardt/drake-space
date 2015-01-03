function out = combineSparse(Q)
%@param Q - cell array of sparse matrices
N = numel(Q);

is = cell(1,N);
js = cell(1,N);
ss = cell(1,N);
for i = 1:N
    [a,b,c] = find(Q{i}); 
   % sz = size(Q{i});
    is{i} = a'; js{i} = b'; ss{i} = c';
end
out = sparse([is{:}],[js{:}],[ss{:}]);
end