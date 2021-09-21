t = out.s.time;

x = out.s.signals.values(:,1);
y = out.s.signals.values(:,2);
z = out.s.signals.values(:,3);

phi = out.s.signals.values(:,4);
theta = out.s.signals.values(:,5);
psi = out.s.signals.values(:,6);

xr = out.ref.signals.values(:,1);
yr = out.ref.signals.values(:,2);
zr = out.ref.signals.values(:,3);

fx = out.f.signals.values(:,1);
fy = out.f.signals.values(:,2);
fz = out.f.signals.values(:,3);

figure(1)

subplot(3,1,1)
plot(t,x,'r')
xlabel('t[s]')
ylabel('x[m]')
hold on; grid on
plot(t,xr,'--')
legend('Actual Trajectory', 'Reference Trajectory')

subplot(3,1,2)
plot(t,y,'b')
xlabel('t[s]')
ylabel('y[m]')
hold on; grid on
plot(t,yr,'--')
legend('Actual Trajectory', 'Reference Trajectory')

subplot(3,1,3)
plot(t,fx,'r')
xlabel('t[s]')
ylabel('F[N]')
hold on; grid on
plot(t,fy,'b')
legend('Fx', 'Fy')
