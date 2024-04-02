close all;

time = XStep1.time;
response = XStep1.Data;

% Find index where command is sent (15 seconds)
commandIndex = find(time >= 15, 1, 'first');

% Adjust the vectors
adjustedTime = time(commandIndex:end) - time(commandIndex);
adjustedResponse = response(commandIndex:end);
stepinfo(adjustedResponse, adjustedTime, 'SettlingTimeThreshold',0.05,'RiseTimeLimits',[0.1,0.9])

% Plot rise time
figure;
risetime(adjustedResponse, adjustedTime, 'StateLevels', [0, 0.25])

% Plot settling time
figure;
settlingtime(adjustedResponse, adjustedTime, 5, 'StateLevels', [0, 0.25], 'Tolerance', 5)

% Combine the time and response data into a two-column matrix
timeSeriesData = [adjustedTime, adjustedResponse];

% Define the filename
filename = 'time_series_data.csv';

% Export the matrix to a CSV file
writematrix(timeSeriesData, filename);
