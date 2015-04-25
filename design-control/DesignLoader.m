classdef DesignLoader
    properties
        folder_template;
        filename = 'variablescmaes.mat';
        runs = 1;
    end
    
    methods
        function obj = DesignLoader(opts)
            %go back and check 
            if nargin > 0
            fields = fieldnames(opts);
       
            for i=1:numel(fields)
                if checkPropertyExists('DesignLoader',fields{i})
                    obj.(fields{i}) = opts.(fields{i});
                end
            end
            end
        end

        function [params, vol, gen] = loadInspectorDesigns(obj)
            vol = [];
            gen = [];
            params = {};
            for i = 1:length(obj.runs)
                foldername = [obj.folder_template,num2str(obj.runs(i))];
                disp(['loading ',foldername]);
                es = load([foldername, '/',obj.filename]);
                if es.out.solutions.bestever.f ~= -10000
                    vol(end+1) = -es.out.solutions.bestever.f;
                    gen(end+1) = obj.runs(i);
                    params{end+1} = es.out.solutions.bestever.x;
                end
            end
        end
        
        function [init_conds] = loadInitialConditions(obj)
            init_conds = {};
            for i = 1:length(obj.runs)
                foldername = [obj.folder_template,num2str(obj.runs(i))];
                disp(['loading ',foldername]);
                es = load([foldername, '/',obj.filename]);
                if es.out.solutions.bestever.f ~= -10000
                    init_conds{end+1} = es.xstart;
                end
            end
        end
        
        function obj = set(obj,property,value)
            if checkPropertyExists('DesignLoader',property)
                obj.(property) = value;
            end
        end
        
        function obj = setFolderTemplate(obj,template_str)
            obj.folder_template = template_str;
        end
    end
end