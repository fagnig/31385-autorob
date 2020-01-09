A2=importdata('line_0_2.dat');
time2 = (1:length(A2(:,1)))/20;
x2 = A2(:,4)
y2 = A2(:,5);

A2B=importdata('line_0_2_B.dat');
time2B = (1:length(A2B(:,1)))/20;
x2B = A2B(:,4)
y2B = A2B(:,5);

A4=importdata('line_0_4.dat');
time4 = (1:length(A4(:,1)))/20;
x4 = A4(:,4)
y4 = A4(:,5);

A4B=importdata('line_0_4_B.dat');
time4B = (1:length(A4B(:,1)))/20;
x4B = A4B(:,4)
y4B = A4B(:,5);

A6=importdata('line_0_6.dat');
time6 = (1:length(A6(:,1)))/20;
x6 = A6(:,4)
y6 = A6(:,5);

A6B=importdata('line_0_6_B.dat');
time6B = (1:length(A6B(:,1)))/20;
x6B = A6B(:,4)
y6B = A6B(:,5);

plot(time2, x2, 'r', 'LineWidth', 4)
hold on
plot(time2B, x2B, '+r')
plot(time4, x4, 'g', 'LineWidth', 4)
plot(time4B, x4B, '+g')
plot(time6, x6, 'b', 'LineWidth', 4)
plot(time6B, x6B, '+b')
xlabel('time (s)')
ylabel('dist (m)')
xlim([0, 11]);
ylim([0, 2.3]);
legend({'0.2 before', '0.2 after', '0.4 before', '0.4 after', '0.6 before', '0.6 after'}, 'Location', 'southeast')
hold off