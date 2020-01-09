A2=importdata('line_0_2.dat');
time2 = (1:length(A2(:,1)))/20;
x2 = A2(:,4);
y2 = A2(:,5);
theta2 = A2(:,6);
% subplot(1,3,1)
% plot(time2, x2, 'r', time2, y2, 'g', time2, theta2, 'b')
title('velocity = 0.2 m/s')
xlabel('time (s)')
ylabel('dist (m)')
legend('x', 'y', 'theta')

A4=importdata('line_0_4.dat');
time4 = (1:length(A4(:,1)))/20;
x4 = A4(:,4);
y4 = A4(:,5);
theta4 = A4(:,6);
% subplot(1,3,2)
% plot(time4, x4, 'r', time4, y4, 'g', time4, theta4, 'b')
title('velocity = 0.4 m/s')
xlabel('time (s)')
ylabel('dist (m)')
legend('x', 'y', 'theta')

A6=importdata('line_0_6.dat');
time6 = (1:length(A6(:,1)))/20;
x6 = A6(:,4);
y6 = A6(:,5);
theta6 = A6(:,6);
% subplot(1,3,3)
% plot(time6, x6, 'r', time6, y6, 'g', time6, theta6, 'b')
title('velocity = 0.6 m/s')
xlabel('time (s)')
ylabel('dist (m)')
legend('x', 'y', 'theta')


plot(time2, x2, 'r')
hold on
plot(time4, x4, 'g')
plot(time6, x6, 'b')
xlabel('time (s)')
ylabel('dist (m)')
xlim([0, 11]);
ylim([0, 2.3]);
legend({'vel = 0.2 m/s', 'vel = 0.4 m/s', 'vel = 0.6 m/s'}, 'Location', 'southeast')
hold off