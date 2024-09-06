% Define the data
%ProximityTaskWeights = [1.4286, 2.5000, 1.4286, 2.5000, 1.4286, 2.5000, 0.4396, 2.5000, 0.4396, 2.5000, 0.4396, 2.5000, 0.4396, 1.2121, 0.4396, 1.2121, 0.4396, 1.2121, 0.4000, 1.2121, 0.4000, 1.2121, 0.4000, 1.2121, 0.6154, 1.0811, 0.6154, 1.0811, 0.6154, 1.0811, 0.6154, 1.0811, 0.6154, 1.0811, 0.6154, 1.0811];
%ProximityTaskDurations = [14.0000, 8.0000, 14.0000, 8.0000, 14.0000, 8.0000, 14.0000, 8.0000, 20.0000, 8.0000, 20.0000, 8.0000, 14.0000, 8.0000, 20.0000, 16.5000, 20.0000, 16.5000, 14.0000, 8.0000, 20.0000, 16.5000, 20.0000, 16.5000, 14.0000, 8.0000, 20.0000, 16.5000, 20.0000, 18.5000, 14.0000, 8.0000, 20.0000, 16.5000, 20.0000, 18.5000];
ProximityTaskWeights
% Create a scatter plot
figure;
scatter(ProximityTaskWeights, ProximityTaskDurations, 'b', 'filled');
xlabel('Proximity Task Weights');
ylabel('Proximity Task Durations');
title('Scatter Plot of Proximity Task Weights vs. Durations');
grid on;

% Calculate the correlation coefficient
correlationCoefficient = corr(ProximityTaskWeights', ProximityTaskDurations');
disp(['Correlation Coefficient: ', num2str(correlationCoefficient)]);

% Optionally, add the correlation coefficient to the plot
annotation('textbox', [0.15, 0.8, 0.3, 0.1], 'String', ['Correlation: ', num2str(correlationCoefficient)], 'FitBoxToText', 'on');
