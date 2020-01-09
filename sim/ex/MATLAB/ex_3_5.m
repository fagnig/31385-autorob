clear; clc; clf

A2=importdata('line_0_2.dat');
time2 = (1:length(A2(:,1)))/20;
x2 = A2(:,4);
y2 = A2(:,5);

A2B=importdata('line_0_2_B.dat');
time2B = (1:length(A2B(:,1)))/20;
x2B = A2B(:,4);
y2B = A2B(:,5);

A2C=importdata('line_0_2_C.dat');
time2C = (1:length(A2C(:,1)))/20;
x2C = A2C(:,4);
y2C = A2C(:,5);

A4=importdata('line_0_4.dat');
time4 = (1:length(A4(:,1)))/20;
x4 = A4(:,4);
y4 = A4(:,5);

A4B=importdata('line_0_4_B.dat');
time4B = (1:length(A4B(:,1)))/20;
x4B = A4B(:,4);
y4B = A4B(:,5);

A4C=importdata('line_0_4_C.dat');
time4C = (1:length(A4C(:,1)))/20;
x4C = A4C(:,4);
y4C = A4C(:,5);

A6=importdata('line_0_6.dat');
time6 = (1:length(A6(:,1)))/20;
x6 = A6(:,4);
y6 = A6(:,5);

A6B=importdata('line_0_6_B.dat');
time6B = (1:length(A6B(:,1)))/20;
x6B = A6B(:,4);
y6B = A6B(:,5);

A6C=importdata('line_0_6_C.dat');
time6C = (1:length(A6C(:,1)))/20;
x6C = A6C(:,4);
y6C = A6C(:,5);

coeffLin = polyfit(time4B, x4B', 1)
xLin = polyval(coeffLin , time4B);
avg_vel = coeffLin(1)

coeffQuad = polyfit(time4C, x4C', 2)
xQuad = polyval(coeffQuad , time4C);
avg_accel = coeffQuad(1)

plot(time4, x4, 'b', 'LineWidth', 0.5)
hold on
plot(time4B, x4B, 'b', 'LineWidth', 1)
plot(time4C, x4C, 'b', 'LineWidth', 2)
plot(time4B, xLin, 'g', 'LineWidth', 2)
plot(time4C, xQuad, 'r', 'LineWidth', 2)
xlim([0, 8.5])
ylim([0, 2.25])
xlabel('time (s)')
ylabel('dist (m)')
legend({'0.4 unlimited', '0.4 accel limited', '0.4 accel/decel limited', ...
    '0.4 accel limited lin fit', '0.4 accel/decel limited quad fit'}, ...
    'Location', 'southeast')
title('Effects of Accel/Decel Limits on Trajectory')
grid on
hold off



plot(time2, x2, 'r', 'LineWidth', 4)
hold on
plot(time2B, x2B, '+r')
plot(time2C, x2C, '.r')
plot(time4, x4, 'g', 'LineWidth', 4)
plot(time4B, x4B, '+g')
plot(time4C, x4C, '.g')
plot(time6, x6, 'b', 'LineWidth', 4)
plot(time6B, x6B, '+b')
plot(time6C, x6C, '.b')
xlabel('time (s)')
ylabel('dist (m)')
xlim([0, 15]);
ylim([0, 2.2]);
legend({'0.2 unlimited', '0.2 accel limited', '0.2 accel/decel limited', ...
    '0.4 unlimited', '0.4 accel limited', '0.4 accel/decel limited', ...
    '0.6 unlimited', '0.6 accel limited', '0.6 accel/decel limited'}, ...
    'Location', 'southeast')
title('Effects of Accel/Decel Limits on Trajectory')
grid on
hold off