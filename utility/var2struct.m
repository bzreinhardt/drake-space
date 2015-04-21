function s = var2struct(varargin)
%turns any number of variables into a struct with fields of those names
  names = arrayfun(@inputname,1:nargin,'UniformOutput',false);
  s = cell2struct(varargin,names,2);
end