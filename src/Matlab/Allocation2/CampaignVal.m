clear all
close all

global indexVal num_agents indexValW

indexCamp = zeros(1,10);
indVal = [];
indValW = [];

for indice=1:5
    HRJournal3HumNumVal;
    indVal = [indVal, indexVal{1}, indexVal{2}, indexVal{3}];
    indValW = [indValW, indexValW{1}, indexValW{2}, indexValW{3}];
    indexCamp(indice) = mean([indexVal{1}, indexVal{2}, indexVal{3}]);
    indexCampW(indice) = mean([indexValW{1}, indexValW{2}, indexValW{3}]);
end

% Plotting the results
figure;

% Plot index values
subplot(2,1,1);
plot(1:10, indexCamp, '-o');
title('Index Values with Reallocation');
xlabel('Iteration');
ylabel('Mean Index Value');
grid on;

% Plot index values
subplot(2,1,2);
plot(1:10, indexCampW, '-o');
title('Index Values without Reallocation');
xlabel('Iteration');
ylabel('Mean Index Value');
grid on;

figure;

% Plot individual index values
subplot(2,1,1);
plot(1:length(indVal), indVal, '-o');
title('Individual Index Values over Iterations with Reallocation');
xlabel('Index');
ylabel('Individual Index Value');
grid on;

% Plot individual index values
subplot(2,1,2);
plot(1:length(indValW), indValW, '-o');
title('Individual Index Values over Iterations without Reallocation');
xlabel('Index');
ylabel('Individual Index Value');
grid on;

% Display the plot
drawnow;
