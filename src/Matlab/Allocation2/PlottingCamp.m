% meansIndexCamp = [2.9786; 2.8027; 2.3384; 2.4685; 2.2642];
% meansIndexCampW = [0.9307; 1.2669; 0.9402; 1.0010; 0.9683];
% meansIndexCamp1 = [4.2285; 4.3999; 2.3488; 1.5658; 1.9085];
% meansIndexCampW1 = [1.2017; 2.2469; 0.5001; 0.4161; 0.8050];
% 
% % Number of values (weights) for each mean
% n1 = 15;  % For meansIndexCamp and meansIndexCampW
% n2 = 5;   % For meansIndexCamp1 and meansIndexCampW1
% 
% % Initialize vectors to store the combined means
% meansIndexCampSum = zeros(size(meansIndexCamp));
% meansIndexCampWSum = zeros(size(meansIndexCamp1));
% 
% % Compute the combined means element-wise using the weighted mean formula
% for i = 1:length(meansIndexCamp)
%     meansIndexCampSum(i) = ((n1 * meansIndexCamp(i)) + (n2 * meansIndexCamp1(i))) / (n1 + n2);
%     meansIndexCampWSum(i) = ((n1 * meansIndexCampW(i)) + (n2 * meansIndexCampW1(i))) / (n1 + n2);
% end
% 
% stdIndexCamp1 = [1.2005; 7.0776; 10.0930; 10; 10];
% stdIndexCampW1 = [25.3067; 3.7392; 6.6867; 10; 10];
% 
% stdIndexCamp = [2.5194; 7.0328; 8.0062; 10; 10]%; 8.5729];
% stdIndexCampW = [1.9466; 2.9176; 2.6047; 10; 10]%; 62.9990];
% 
% % Sum of the vectors
% meansIndexCampSum = meansIndexCamp1 + 15*meansIndexCamp;
% meansIndexCampWSum = meansIndexCampW1 + 15*meansIndexCampW;
% 
% % Sum of the standard deviations (assuming independence)
% stdIndexCamp = [1.0398; 1.8042; 1.2979; 1.6975];
% stdIndexCampW = [0.3874; 1.2172; 0.8597; 0.5861];
% 
% meansIndexCamp = [2.9786; 2.8027; 2.4034; 2.2642];
% meansIndexCampW = [0.9307; 1.2669; 0.9706; 0.9683];

stdIndexCamp = [1.3065; 1.7741; 1.4764; 1.4964; 0];
stdIndexCampW = [0.2890; 1.1845; 0.9640; 0.9119; 0];

meansIndexCamp = [3.3991; 2.6735; 2.4146; 2.3208; 0.6184];
meansIndexCampW = [1.0931; 1.0024; 1.02778; 1.1097; -0.5574];

% Plotting the mean results with std error bars
num_scenarios = length(meansIndexCamp);

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
