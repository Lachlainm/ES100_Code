% Parameters
Q = 0.6; % hr^-1
K = 0.005; % g/L

% Time span
t = linspace(24, 1000, 1000); % 1000 points between 0 and 24 hours for a smooth plot

% Solution for y(t) as a function handle
y_t = @(t) (10*Q)/K - 1353 .* exp(-K * t);

% Calculate X(t) by taking the reciprocal of y(t)
y_values = y_t(t);

% Plot X(t)
figure;
plot(t, y_values, 'LineWidth', 2);
xlabel('Time (hours)');
ylabel('Accumulation of Cycloheximide');
grid on;
