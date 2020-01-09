raw_data = importdata('log');

[num_rows, num_cols] = size(raw_data);
num_samples = num_rows;
num_sensors = num_cols - 1;

states = raw_data(:, 1);
black_state = 1;    % state = 1 --> black --> 0-20.4875
white_state = 2;    % state = 2 --> white --> 1

num_black = 0;
black_sums = zeros(1, num_sensors);
num_white = 0;
white_sums = zeros(1, num_sensors);

for i = 1:num_samples
    if states(i) == black_state
        num_black = num_black + 1;
        black_sums = black_sums + raw_data(i, 2:end);
    end
    if states(i) == white_state
        
        num_white = num_white + 1;
        white_sums = white_sums + raw_data(i, 2:end);
    end       
end

% todo: fix division by zero vulnerabilities
black_avg = black_sums/num_black;
white_avg = white_sums/num_white;

%avg_limits = [black_avg; white_avg];

slopes = 1 ./ (white_avg - black_avg);
intercepts = -black_avg .* slopes;
coeffs = [slopes; intercepts];

scaled_data = zeros(num_rows, num_cols);
scaled_data(:,1) = states;

for i = 1:num_samples
    if ((states(i) ~= black_state) && (states(i) ~= white_state))
        %scaled_data(i, 2:end) = -1*ones(1,num_sensors);
        scaled_data(i, 2:end) = -Inf;
    else
        scaled_data(i, 2:end) = slopes .* raw_data(i, 2:end) + intercepts;
        scaled_data(i, 2:end) = max(0, min(1, scaled_data(i, 2:end)));
    end       
end

coeffs
scaled_data