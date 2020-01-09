A=importdata('output.dat');
mission_times = A(:,1)
time = 1:length(mission_times);
vel_r = A(:,2);
vel_l = A(:,3);
plot(time, vel_r, 'b', 'LineWidth', 4)
hold on
plot(time, vel_l, '-r', 'LineWidth', 1.5)
xlabel('time (s)')
ylabel('velocity (m/s)')
legend('right motor', 'left motor')
xlim([-10, 500])
ylim([-0.2, 0.4])
hold off