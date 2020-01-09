A=importdata('output.dat');
x = A(:,4);
y = A(:,5);
plot(x, y, 'b', 'LineWidth', 4)
xlabel('x (m)')
ylabel('y (m)')
% xlim([-10, 500])
% ylim([-0.2, 0.4])