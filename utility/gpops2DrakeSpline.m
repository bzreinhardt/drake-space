function [xtraj, utraj] = gpops2DrakeSpline(obj,output,options)

if nargin > 2
end
%get gpops stuff in right form 
% Extract Solution. 
if isfield(output,'result')
solution = output.result.solution;
elseif isfield(output,'solution')
        solution = output.solution;
elseif isfield (output,'phase');
    solution = output;
elseif isfield(output, 'time')
    solution.phase = output;
end
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
      
    utraj = PPTrajectory(foh(t,u));
      utraj = utraj.setOutputFrame(obj.getInputFrame);

if nargout < 2
    out.xtraj = xtraj;
    out.utraj = utraj;
    xtraj = out;
end
end
