% clear all
% close all
% 
% global indexVal num_agents indexValW
% indVal = [];
% indValW = [];
% indexCamp = [];
% indexCampW = [];
% 
% indexCamp = [indexCamp, zeros(1, 10)]; % Append zeros to extend the list to accommodate indices 11 to 20
% indexCampW = [indexCampW, zeros(1, 10)]; % Similarly extend indexCampW


for indice = 1:10 
    HRJournal3HumNumVal;
    
    % Update the index values for each iteration
    indVal = [indVal, indexVal{1}, indexVal{2}, indexVal{3}, indexVal{4}];
    indValW = [indValW, indexValW{1}, indexValW{2}, indexValW{3}, indexValW{4}];
    
    % Compute the mean index values and store them
    indexCamp(indice) = mean([indexVal{1}, indexVal{2}, indexVal{3}, indexVal{4}]);
    indexCampW(indice) = mean([indexValW{1}, indexValW{2}, indexValW{3}, indexValW{4}]);
    indice
end

% Plotting the results
figure;

% Plot index values with reallocation
plot(1:10, indexCamp(1:10), '-o', 'DisplayName', 'With Reallocation');
hold on;

% Plot index values without reallocation
plot(1:10, indexCampW, '-x', 'DisplayName', 'Without Reallocation');

% Add title and labels
title('Index Values with and without Reallocation');
xlabel('Iteration');
ylabel('Mean Index Value');
grid on;

% Add a legend to distinguish the two plots
legend;

% Display the plot
drawnow;

% Plot individual index values
figure;
plot(1:length(indVal), indVal, '-o', 'DisplayName', 'Individual Index Values with Reallocation');
hold on;
plot(1:length(indValW), indValW, '-x', 'DisplayName', 'Individual Index Values without Reallocation');

% Add title and labels
title('Individual Index Values over Iterations');
xlabel('Index');
ylabel('Individual Index Value');
grid on;

% Add a legend to distinguish the two plots
legend;

% Display the plot
drawnow;
