function symbolic_gradients(fname,order,varargin)
%varargin {1} needs to be a PlanarBody Inspector
%2D things

x = sym('x');
y = sym('y');
theta = sym('theta');
vx = sym('vx');
vy = sym('vy');
omega = sym('omega');
for j=1:size(d,1)
      u(j)=sym(['u',num2str(j)]);
end

X = [x y theta vx vy omega];
c = [0;-5];
r = 5;
d = [1/2^.5 -1/2^.5; -1/2^.5 -1/2^.5]; %replace with object property

v = [vx;vy];
F_tot = 0;
for i = 1:size(d,1)
    %arm in body coordinates
    d_b = d(i,:)';
    d_w = [cos(theta) -sin(theta); sin(theta) cos(theta)]*d_b + [x;y];
    g = sqrt(norm((d_w - c))^2 - r^2);
    n = (d_w-c)/norm(d_w-c);
    Fx = (-5.6219e-08.*u(i).^3+6.2003e-06.*u(i).^2+76.0622.*u(i))...
        .*6.2003e-06.*exp(-44.1463.*g);
    Fy = (5.811e-09.*u(i).^4+-2.2676e-08.*u(i).^3+-1.8637...
        .*u(i).^2+0.29795.*u(i)).*-33.8115.*exp(-38.9901.*g+-21.8594);
    F = [-Fx*n(2) + Fy*n(1); ...
        Fx*n(1) + Fy*n(2); ...
        -d_w(2)*(-Fx*n(2)+Fy*n(1))+d_w(1)*(Fx*n(1) + Fy*n(2))];
    F_tot = F_tot+F;
end

dFdu = [sym(zeros(3,size(d,2)));jacobian(F_tot,u)];
dFdx = [sym(zeros(3,6));jacobian(F,X)];
dFdt = sym(zeros(6,1));
  
%find the gradient of the state-transition matrix 
% Inertia and mass are 1 for now
dAdx = sym([[zeros(3,3),eye(3)];...
    zeros(3,6)]);
dAdu = sym(zeros(6,2));
dAdt = sym(zeros(6,1));

df{1} = [dAdt+dFdt,dAdx+dFdx,dAdu+dFdu];

%% write file
% strip off .m from filename if it came in that way

ind=find(fname=='.',1);
if (~isempty(ind)) fname=fname(1:(ind-1)); end
mfile = sprintf('%s.m',fname);



% open file and write header
fp = fopen(mfile,'w');
fprintf(fp,'function [df');
for o=2:order %write highter order ouptuts
  fprintf(fp,', d%df',o);
end
fprintf(fp,'] = %s(a1',fname); %actual function name
for i=2:length(varargin)
  fprintf(fp,', a%d',i);
end
fprintf(fp,', order)\n');
fprintf(fp,'%% This is an auto-generated file.\n%%\n%% See <a href="matlab: help symbolic_gradients">symbolic_gradients</a>. \n\n');

% check inputs
fprintf(fp,'%% Check inputs:\n');

if (isobject(varargin{1}))
  fprintf(fp,'typecheck(a1,''%s'');\n',class(varargin{1}));
end
fprintf(fp,'if (nargin<%d) order=1; end\n',length(varargin));
fprintf(fp,'if (order<1) error('' order must be >= 1''); end\n');
for i=1:length(varargin)
  fprintf(fp,'sizecheck(a%d,[%s]);\n',i,num2str(size(varargin{i})));
end

% write symbols
fprintf(fp,'\n%% Symbol table:\n');
fprintf(fp,'d = a1.d;\n');
fprintf(fp,'c = a1.sphere_center;\n');
fprintf(fp,'r = a1.sphere_radius;\n');
fprintf(fp,'x = a2(1);');
fprintf(fp,'y = a2(2);');
fprintf(fp,'theta = a2(3);');
fprintf(fp,'vx = a2(4);');
fprintf(fp,'vy = a2(5);');
fprintf(fp,'omega = a2(6);');

fprintf(fp,'\n\n%% Compute Gradients:\n');
write_symbolic_matrix(fp,df{1},'df');

fprintf(fp,'\n\n %% NOTEST\n'); 

% close file
fclose(fp);


end

function write_symbolic_matrix(fp,A,symbol)
  [m,n]=size(A);
  fprintf(fp,'%s = sparse(%d,%d);\n',symbol,m,n);
  for i=1:m, for j=1:n, if (A(i,j)~=0) fprintf(fp,'%s(%d,%d) = %s;\n',symbol,i,j,char(A(i,j))); end; end; end
end


    






