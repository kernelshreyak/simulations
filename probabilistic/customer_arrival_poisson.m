lambda = 10; % Average rate (customers per hour)
n = 100; % Simulate n arrivals
inter_arrival_times = -log(rand(1, n)) / lambda; % Generate inter-arrival times
arrival_times = cumsum(inter_arrival_times); % Cumulative arrival times

% Plot the arrival times
stairs(arrival_times, 1:n, 'LineWidth', 2);
xlabel('Time (hours)');
ylabel('Customer Number');
title('Simulated Customer Arrivals');
grid on;
