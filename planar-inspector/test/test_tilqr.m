

%Test drake TILQR
p = Inspector2d;

%%%%%%%%%% Some sample optimal control and actual position data %%%%%%%%
x_nom = [0;0.11;0;0;0;0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

[c,V] = findLQR(p,x_nom);
vol = V.getLevelSetVolume();

xnoise = 0.001;
dxnoise = 0.001;
cov = blkdiag(xnoise*eye(3),dxnoise*eye(3));

sys = feedback(p,c);
v = Inspector2dVisualizer(p);

xtraj = simulate(sys,[0 200],x_nom+[0.00;0.0;0;0;0.001;0]);
utraj = output(c, [],[],xtraj);
figure(123);
fnplt(utraj,1);
hold on;
fnplt(utraj,2);
fnplt(utraj,3);
hold off;
% plot the states
figure(99); clf;
subplot(211);

fnplt(xtraj,2);
hold on;
fnplt(xtraj,1);
hold off;
subplot(212)
fnplt(xtraj,4);
hold on;
fnplt(xtraj,5);
hold off;

figure(100);clf;
subplot(211);
fnplt(xtraj,3);
subplot(212);
fnplt(xtraj,6);

%v.playback(xtraj);

%%%%%%%%%%%%%%%%%%%%%
