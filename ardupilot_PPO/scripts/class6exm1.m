
% Initial condition
y0 = 1;

% Time span
tspan = [0, 1.2];

% Solve the ODE numerically using ode45
[t, y] = ode45(@ode_func, tspan, y0);

% Plot the numerical solution
plot(t, y, 'LineWidth', 2);
hold on;

% Plot the approximate solution
t_approx = linspace(0, 1.2, 100);
y_approx = (- 1 ./ (t_approx + 1)).^(3/5);
plot(t_approx, y_approx, '--', 'LineWidth', 2);

xlabel('t');
ylabel('y(t)');
legend('Numerical Solution', 'Approximate Solution');
title('Comparing Numerical and Approximate Solutions');
grid on;
hold off;


function dydt = ode_func(t, y)
    dydt = y^(5/3) + (1 / (t + 1));
end
