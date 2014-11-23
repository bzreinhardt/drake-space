% Extract Solution.                       
solution = output.result.solution;
time = solution.phase.time;
state = solution.phase.state;
control = solution.phase.control;
realControl = [control(:,1),control(:,2)];
for i=1:length(output.meshhistory);
  mesh(i).points = [0 cumsum(output.meshhistory(i).result.setup.mesh.phase.fraction)];
  mesh(i).iteration = i*ones(size(mesh(i).points));
end;

% Plot Solution.
figure(1);
pp = plot(time,state(:,1),'-o');
xl = xlabel('t [sec]');
yl = ylabel('x [m]');
set(xl,'Fontsize',18);
set(yl,'Fontsize',18);
set(gca,'Fontsize',16);
set(pp,'LineWidth',1.25);
%axis square
grid on
%print -dpng Time-vs-xr.png

figure(2);
pp = plot(time,state(:,2),'-o');
xl = xlabel('t [sec]');
yl = ylabel('y [m]');
set(xl,'Fontsize',18);
set(yl,'Fontsize',18);
set(gca,'Fontsize',16);
set(pp,'LineWidth',1.25);
%axis square
grid on
%print -dpng Time-vs-yr.png

figure(3);
pp = plot(time,state(:,3),'-o');
xl = xlabel('t [sec]');
yl = ylabel('thetd [rad]');
set(xl,'Fontsize',18);
set(yl,'Fontsize',18);
set(gca,'Fontsize',16);
set(pp,'LineWidth',1.25);
%axis square
grid on
%print -dpng Time-vs-thetar.png

figure(4);
pp = plot(time,state(:,4),'-o');
xl = xlabel('t [sec]');
yl = ylabel('vx [m/s]');
set(xl,'Fontsize',18);
set(yl,'Fontsize',18);
set(gca,'Fontsize',16);
set(pp,'LineWidth',1.25);
%axis square
grid on
%print -dpng Time-vs-Vxr.png

figure(5);
pp = plot(time,state(:,5),'-o');
xl = xlabel('t [sec]');
yl = ylabel('vy [m/s]');
set(xl,'Fontsize',18);
set(yl,'Fontsize',18);
set(gca,'Fontsize',16);
set(pp,'LineWidth',1.25);
%axis square
grid on
%print -dpng Time-vs-Vyr.png

figure(6);
pp = plot(time,state(:,6),'-o');
xl = xlabel('t [sec]');
yl = ylabel('omega [rad/s]');
set(xl,'Fontsize',18);
set(yl,'Fontsize',18);
set(gca,'Fontsize',16);
set(pp,'LineWidth',1.25);
%axis square
grid on
%print -dpng Time-vs-angVel.png

figure(7);
pp = plot(time,realControl(:,1),'-o');
xl = xlabel('t [sec]');
yl = ylabel('T1');
set(xl,'Fontsize',18);
set(yl,'Fontsize',18);
set(gca,'Fontsize',16);
set(pp,'LineWidth',1.25);
%axis square
grid on
%print -dpng Time-vs-Control.png

figure(8);
pp = plot(time,realControl(:,2),'-o');
xl = xlabel('t [sec]');
yl = ylabel('T2');
set(xl,'Fontsize',18);
set(yl,'Fontsize',18);
set(gca,'Fontsize',16);
set(pp,'LineWidth',1.25);
%axis square
grid on
%print -dpng Time-vs-Control.png

