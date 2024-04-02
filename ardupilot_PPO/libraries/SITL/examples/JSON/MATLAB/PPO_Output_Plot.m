close all;

% Load your data here
% PPO_Angles, PPO_Position, and PPO_Tilt_Angles should be loaded from your Simulink export

% Set the time range for plotting
startTime = 106;
endTime = 110; % Adjust as needed

% Create a figure with three subplots
figure;

% Plot PPO_Angles (Roll, Pitch, Yaw) in the first subplot
timeAngles = PPO_Angle.Time;
anglesData = PPO_Angle.Data;
startIndexAngles = find(timeAngles >= startTime, 1, 'first');
endIndexAngles = find(timeAngles >= endTime, 1, 'first');
subplot(1, 3, 1);
plot(timeAngles(startIndexAngles:endIndexAngles) - timeAngles(startIndexAngles), anglesData(startIndexAngles:endIndexAngles, :));
title('PPO Angles: Roll, Pitch, Yaw');
xlabel('Time (s)');
ylabel('Angles (deg)');
legend('Roll', 'Pitch', 'Yaw');

% Plot PPO_Position (X, Y, Z) in the second subplot
timePosition = PPO_Position.Time;
positionData = PPO_Position.Data;
startIndexPosition = find(timePosition >= startTime, 1, 'first');
endIndexPosition = find(timePosition >= endTime, 1, 'first');
subplot(1, 3, 2);
plot(timePosition(startIndexPosition:endIndexPosition) - timePosition(startIndexPosition), positionData(startIndexPosition:endIndexPosition, :));
title('PPO Position: X, Y, Z');
xlabel('Time (s)');
ylabel('Position (m)');
legend('X', 'Y', 'Z');

% Plot PPO_Tilt_Angles (8 servo motors) in the third subplot
timeTiltAngles = PPO_Tilt_Angles.Time;
tiltAnglesData = PPO_Tilt_Angles.Data;
startIndexTiltAngles = find(timeTiltAngles >= startTime, 1, 'first');
endIndexTiltAngles = find(timeTiltAngles >= endTime, 1, 'first');
subplot(1, 3, 3);
plot(timeTiltAngles(startIndexTiltAngles:endIndexTiltAngles) - timeTiltAngles(startIndexTiltAngles), tiltAnglesData(startIndexTiltAngles:endIndexTiltAngles, :));
title('PPO Tilt Angles: 8 Servo Motors');
xlabel('Time (s)');
ylabel('Angles (rad)');
legend(arrayfun(@(x) ['Servo ' num2str(x)], 1:8, 'UniformOutput', false));
