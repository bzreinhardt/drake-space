function [xtraj, utraj] = gpops2DrakeSpline(obj,output,options)

if nargin > 2
end
%get gpops stuff in right form 
% Extract Solution.                       
solution = output.result.solution;
t = solution.phase.time';
x = solution.phase.state';
u = solution.phase.control';

N = length(t);

%find the time derivatives at each point
 xdot = zeros(size(x,1),N);
for i=1:N,
        xdot(:,i) = obj.dynamics(t(i),x(:,i),u(:,i));
end
%
xtraj = PPTrajectory(pchipDeriv(t,x,xdot));
      xtraj = xtraj.setOutputFrame(obj.getStateFrame);
end