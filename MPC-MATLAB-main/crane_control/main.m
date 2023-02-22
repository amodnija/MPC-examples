clear all; close all; clc
addpath('apm')

% server and application
s = 'http://byu.apmonitor.com';
a = 'crane_pendulum';

% clear application
apm(s,a,'clear all');

% load model and data file
apm_load(s,a,'pendulum.apm');
csv_load(s,a,'pendulum.csv');

% option: dynamic control mode
apm_option(s,a,'nlc.imode',6);

% classify: manipulated variable
apm_info(s,a,'MV','u');

% option: let optimizer use MV to minimize objective
apm_option(s,a,'u.status',1);

% solve
output = apm(s,a,'solve');
disp(output)

% retrieve results
y = apm_sol(s,a);
z = y.x;

figure(1)
subplot(4,1,1)
plot(z.time,z.u,'r-','LineWidth',2)
ylabel('Force on Cart')
legend('u')
axis tight

subplot(4,1,2)
plot(z.time,z.y,'b--','LineWidth',2)
ylabel('Cart Position')
legend('y')
axis tight

subplot(4,1,3)
plot(z.time,z.v,'g:','LineWidth',2)
ylabel('Cart Velocity')
legend('v')
axis tight

subplot(4,1,4)
plot(z.time,z.theta,'m.-','LineWidth',2)
hold on
plot(z.time,z.q,'k.','LineWidth',2)
ylabel('Pendulum Mass')
legend('\theta (Angle)','q (Angle Rate)')
axis tight
xlabel('Time')



