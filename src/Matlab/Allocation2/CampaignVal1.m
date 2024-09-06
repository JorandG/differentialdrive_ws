% clear all
% close all
% 
% global indexVal num_humans indexValW num_robots num_scenarios
% indVal = [];
% indValW = [];
% indexCamp = [];
% indexCampW = [];
% 
% % Define the scenarios
% scenarios = [
%     2, 2;  % 2 humans, 2 robots
%     3, 2;  % 3 humans, 2 robots
%     4, 2;  % 4 humans, 2 robots
%     5, 2   % 5 humans, 2 robots
%     %6, 2   % 6 humans, 2 robots
% ];
% 
% num_scenarios = size(scenarios, 1);
% 
% % Preallocate for storing mean and std values
% meansIndexCamp = zeros(num_scenarios, 1);
% meansIndexCampW = zeros(num_scenarios, 1);
% stdIndexCamp = zeros(num_scenarios, 1);
% stdIndexCampW = zeros(num_scenarios, 1);

%for scenario_idx = 1:num_scenarios
    % Set the number of humans and robots for this scenario
    num_humans = scenarios(scenario_idx, 1);
    num_robots = scenarios(scenario_idx, 2);
    
    % Reset variables that are scenario-dependent
    indexCamp = [];
    indexCampW = [];
    indVal = [];
    indValW = [];

    indice = 1;  % Initialize indice to start at 1
    while indice <= 25  % Iterate through 30 repetitions for statistical stability
        try
            % Run the simulation with current scenario settings
            HRJournalXHumXRobNumVal;
            
            % Update the index values for each iteration
            for agent_idx = 1:num_humans
                indVal = [indVal, indexVal{agent_idx}];
                indValW = [indValW, indexValW{agent_idx}];
            end
            
            % Compute the mean index values and store them for this iteration
            indexCamp(indice) = mean(cell2mat(indexVal(1:num_humans)));
            indexCampW(indice) = mean(cell2mat(indexValW(1:num_humans)));
            
            % Save the workspace after each successful indice
            save_filename = sprintf('workspace_scenarios10.mat', scenario_idx, indice);
            save(save_filename);
            
            % Move to the next indice if successful
            indice = indice + 1;
            
        catch ME
            % Log a warning message with the error
            warning('An error occurred during scenario %d, iteration %d: %s. Retrying...', scenario_idx, indice, ME.message);
            % The loop will retry the same `indice` until successful
        end
    end
    
    % Store the mean and std across all iterations for this scenario
    meansIndexCamp(scenario_idx) = mean(indexCamp);
    meansIndexCampW(scenario_idx) = mean(indexCampW);
    stdIndexCamp(scenario_idx) = std(indexCamp);
    stdIndexCampW(scenario_idx) = std(indexCampW);
%end

% Plotting the mean results with std error bars
figure;
hold on;

% Plot mean and std for indexCamp (with reallocation)
errorbar(1:num_scenarios, meansIndexCamp, stdIndexCamp, '-o', 'DisplayName', 'With Reallocation');

% Plot mean and std for indexCampW (without reallocation)
errorbar(1:num_scenarios, meansIndexCampW, stdIndexCampW, '-x', 'DisplayName', 'Without Reallocation');

% Add title and labels
title('Mean Index Values with and without Reallocation across Scenarios');
xlabel('Scenario');
ylabel('Mean Index Value');
xticks(1:num_scenarios);
xticklabels({'2H2R', '3H2R', '4H2R', '5H2R'});
grid on;

% Add a legend to distinguish the two plots
legend;

% Display the plot
drawnow;

% Plot individual index values for each scenario
figure;
hold on;
for scenario_idx = 1:num_scenarios
    plot(repmat(scenario_idx, 1, length(indVal)), indVal, 'o', 'DisplayName', ['Scenario ' num2str(scenario_idx) ' With Reallocation']);
    plot(repmat(scenario_idx, 1, length(indValW)), indValW, 'x', 'DisplayName', ['Scenario ' num2str(scenario_idx) ' Without Reallocation']);
end

% Add title and labels
title('Individual Index Values over Scenarios');
xlabel('Scenario');
ylabel('Individual Index Value');
xticks(1:num_scenarios);
xticklabels({'2H2R', '3H2R', '4H2R', '5H2R'});
grid on;

% Add a legend to distinguish the two plots
legend;

% Display the plot
drawnow;
