function [xtraj, utraj] = gpops2DrakeSpline(obj,t,x,u)
N = length(t);
%get gpops stuff in right form 
for i=1:N,
        xdot(:,i) = obj.dynamics(t(i),x(:,i),u(:,i));
      end
xtraj = PPTrajectory(pchipDeriv(t,x,xdot));
      xtraj = xtraj.setOutputFrame(obj.getStateFrame);
end