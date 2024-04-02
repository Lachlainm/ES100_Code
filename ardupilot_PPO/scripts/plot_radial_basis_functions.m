% Define parameters for radial basis functions
num_basis_functions = 20;
centers = linspace(-1.5, 1.5, num_basis_functions); % Centers of RBFs
sigma = 0.08; % Spread parameter for RBFs

% Define the range of input values to plot over
input_range = linspace(-1, 1, 1000); % Modify this range as needed

% Initialize matrix to store RBF outputs
RBF_outputs = zeros(num_basis_functions, length(input_range));

% Evaluate each RBF over the input range
for i = 1:num_basis_functions
    for j = 1:length(input_range)
        RBF_outputs(i, j) = exp(-(norm(input_range(j) - centers(i))^2) / (2 * sigma^2));
    end
end

% Plot the RBFs
figure;
hold on;
for i = 1:num_basis_functions
    plot(input_range, RBF_outputs(i, :));
end
title('Radial Basis Function Outputs');
xlabel('Input Value');
ylabel('RBF Output');
hold off;
