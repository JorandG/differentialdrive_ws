clear all
close all

global indexVal num_agents indexValW
indVal = [];
indValW = [];
indexCamp = [];
indexCampW = [];

indexCamp = [indexCamp, zeros(1, 10)]; % Append zeros to extend the list to accommodate indices 11 to 20
indexCampW = [indexCampW, zeros(1, 10)]; % Similarly extend indexCampW


for indice = 1:10 
    HRJournalXHumXRobNumVal;
    
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


% Assuming num_agents is the total number of agents
num_agents = length(storedIndexVal); % Or set this value accordingly

% Define the number of iterations (assuming all agents have the same number of iterations)
num_iterations = length(storedIndexVal{1});

% Create a colormap with two distinct colors for two iterations
colors = lines(2);

% Create the first figure for storedIndexVal
figure;

% Initialize arrays to store plot handles for the legend
h = gobjects(2, 1);

% Plot each point for storedIndexVal
for iter = 1:num_iterations
    for u = 1:num_agents
        h(iter) = plot(u, indexValMatrix(u, iter), 'o', 'MarkerSize', 8, 'MarkerFaceColor', colors(iter, :), 'MarkerEdgeColor', 'black');
        hold on;
    end
end

% Customize the plot
xlabel('Agent');
ylabel('Stored Index Value');
title('Stored Index Values (indexVal) for All Agents');
xlim([0, num_agents + 1]);
grid on;

% Add a legend corresponding to the two iterations
legend(h, {'Iteration 1', 'Iteration 2'}, 'Location', 'Best');

% Create the second figure for storedIndexValW
figure;

% Initialize arrays to store plot handles for the legend
h = gobjects(2, 1);

% Plot each point for storedIndexValW
for iter = 1:num_iterations
    for u = 1:num_agents
        h(iter) = plot(u, indexValWMatrix(u, iter), 'o', 'MarkerSize', 8, 'MarkerFaceColor', colors(iter, :), 'MarkerEdgeColor', 'black');
        hold on;
    end
end

% Customize the plot
xlabel('Agent');
ylabel('Stored Index Value');
title('Stored Index Values (indexValW) for All Agents');
xlim([0, num_agents + 1]);
grid on;

% Add a legend corresponding to the two iterations
legend(h, {'Iteration 1', 'Iteration 2'}, 'Location', 'Best');

