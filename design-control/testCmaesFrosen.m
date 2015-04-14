function testCmaesFrosen
%check that cmaes works with frosen function
   function f=frosenbrock(x)
       %for N = 2, global minum at (x,y) = (a,a^2)
       %f(x,y) = (a-x)^2 + b(y-x^2)^2
        if size(x,1) < 2 error('dimension must be greater one'); end
        f = 100*sum((x(1:end-1).^2 - x(2:end)).^2) + sum((x(1:end-1)-1).^2);
   end

x0 = [0;0];
sigma = 1.5;

xmin = cmaes('frosenbrock',x0,sigma);
disp(xmin);
end

