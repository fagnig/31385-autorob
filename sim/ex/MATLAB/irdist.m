function irout = irdist(k, d)

raw_data = importdata('IRlog');

[num_rows, num_cols] = size(raw_data);
num_samples = num_rows;
num_sensors = num_cols - 1;

num_meas = 0;
for i = 1:num_samples
    if raw_data(i,1) ~= -1
        num_meas = num_meas + 1;
        distances(num_meas) = raw_data(i,1);
        readings(num_meas, 1:num_sensors) = raw_data(i,2:end); 
    end
end

coeffs_guess = [16,10];

distances = distances';
mapping = @(x,distances)x(1)./distances+x(2);


readings1 = readings(:,1);
coeffs_fit1 = lsqcurvefit(mapping,coeffs_guess,distances,readings1);

readings2 = readings(:,2);
coeffs_fit2 = lsqcurvefit(mapping,coeffs_guess,distances,readings2);

readings3 = readings(:,2);
coeffs_fit3 = lsqcurvefit(mapping,coeffs_guess,distances,readings3);

coeffs = [coeffs_fit1', coeffs_fit2', coeffs_fit3']

size(readings)

predicted_dist = ones(719,1)*coeffs(1,:)./(readings - ones(719,1)*coeffs(2,:));

[distances, predicted_dist]

x = linspace(min(distances),max(distances));

subplot(1,3,1)
plot(distances, predicted_dist(:,1), 'Linewidth', 2')
hold on
plot(distances, distances, 'x','Linewidth', 2')
xlim([20,90])
ylim([20,90])
xlabel('Actual Distances (cm)')
ylabel('Predicted Distances (cm)')
title('Front Left Sensor')
legend({'Model', 'Real'}, 'Location', 'best')
% plot(distances, (distances - predicted_dist(:,1))./distances*100)
% ylim([-50,50])
grid on
hold off

subplot(1,3,2)
plot(distances, predicted_dist(:,2), 'Linewidth', 2')
hold on
plot(distances, distances, 'x','Linewidth', 2')
xlim([20,90])
ylim([20,90])
xlabel('Actual Distances (cm)')
ylabel('Predicted Distances (cm)')
title('Front Middle Sensor')
legend({'Model', 'Real'}, 'Location', 'best')
% plot(distances, (distances - predicted_dist(:,1))./distances*100)
% ylim([-50,50])
grid on
hold off

subplot(1,3,3)
plot(distances, predicted_dist(:,3), 'Linewidth', 2')
hold on
plot(distances, distances, 'x','Linewidth', 2')
xlim([20,90])
ylim([20,90])
xlabel('Actual Distances (cm)')
ylabel('Predicted Distances (cm)')
title('Front Right Sensor')
legend({'Model', 'Real'}, 'Location', 'best')
% plot(distances, (distances - predicted_dist(:,1))./distances*100)
% ylim([-50,50])
grid on
hold off

irdist = -1;