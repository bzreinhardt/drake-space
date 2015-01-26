function params = generateParams(template,options)
%GENERATEPARAMS generates parameters
%@param template - a template matrix for the size and types of the output
%@options - min - matrix of minimum values - 
%@options - max - matrix of maximum values
%@options - method - method of picking parameters: 'rand',
%options - constraints



%Inspector Parameters
%remain in a 0.1 m sphere
%4 couplers - keep this invariant for now

params = zeros(size(template));

if nargin<1
    options = struct();
end
%check options, assign defaults
if isfield(options,'method')
    method = options.method;
else
    method = 'rand';
end
if isfield(options,'max');
    max = options.max;
else
    max = ones(size(template));
end
if isfield(options,'min');
    min = options.min;
else
    min = zeros(size(template));
end


if strcmp('rand',method)
    for i = 1:numel(template)
        params(i) = (max(i)-min(i))*rand(1)+min(i);
    end
else
end



end