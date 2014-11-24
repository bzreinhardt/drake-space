function trajectory_plot(output,options)
%Defaults
print_on = 0;
directory = '';
save = 0;
disp = 1;
animate = 0;
if nargin > 1
if isfield(options,'print_on')
    print_on = options.print_on;
end
if isfield(options,'dir')
    directory = options.dir;
end
if isfield(options,'save')
    save = options.save;
end
if isfield(options,'disp')
    disp = options.disp;
end
if isfield(options, 'animate')
    animate = options.animate;
end
end
    
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
if (animate)
    figure(1337);clf;
    rectangle('Position',[-5,-10,10,10],'Curvature',[1 1],'FaceColor',[150 150 150]/255);
    drawnow;
    hold on;
    lower = min(min(state(:,1)),min(state(:,2)))-0.1;
    upper = max(max(state(:,1)),max(state(:,2)))+0.1;
    xlim([lower,upper]);
    ylim([lower,upper]);
for i = 1:size(state,1)

    if (i~=1)
        delete(h);
    end
    h = drawInductionInspector2D(state(i,:));
    drawnow;
end
end

if (disp)
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
yl = ylabel('U1');
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
yl = ylabel('U2');
set(xl,'Fontsize',18);
set(yl,'Fontsize',18);
set(gca,'Fontsize',16);
set(pp,'LineWidth',1.25);
%axis square
grid on
%print -dpng Time-vs-Control.png
end
% --------------------------------------------------------------%
%           Print Things                                        %
% --------------------------------------------------------------%
if (print_on)
    fig_names = {'x','y','theta','vx','vy','omega','u1','u2'};
    run_name = strcat('gpops_traj_',date,output.name);
    for i = 1:8
    figure(i);
    save2pdf(strcat(directory,'/',run_name,fig_names{i}));
    end
end

% --------------------------------------------------------------%
%           Save Things                                         %
% --------------------------------------------------------------%
