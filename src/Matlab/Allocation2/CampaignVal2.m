% Assuming meansIndexCamp, meansIndexCampW, stdIndexCamp, and stdIndexCampW are already calculated.
% meansIndexCampW = [0.2464; 2.9907; 0.7628; -5.5920];
% meansIndexCamp = [3.7603; 7.5495; 6.3475; 7.19042];
% 
% stdIndexCamp = [1.9557; 9.0971; 12.8660; 10.8225];
% stdIndexCampW = [2.8740; 4.7275; 17.4484; 40.0838];

meansIndexCampW = [0.2464; 2.9907; 7.4439; 7.9587];
meansIndexCamp = [3.7603; 7.5495; 11.1009; 6.7782];

stdIndexCamp = [1.9557; 9.0971; 12.2531; 8.8390];
stdIndexCampW = [2.8740; 4.7275; 7.7646; 6.0391];

% Calculate the difference of the means
diffMeans = abs(meansIndexCampW - meansIndexCamp);

% Calculate the combined standard deviation assuming independence
diffStd = sqrt(stdIndexCamp.^2 + stdIndexCampW.^2);

% Plotting the difference of means with combined std error bars
figure;
hold on;

% Plot the difference of means with error bars
errorbar(1:4, diffMeans, diffStd, '-o', 'DisplayName', 'Difference (With - Without Reallocation)');

% Add title and labels
title('Difference of Mean Index Values across Scenarios');
xlabel('Scenario');
ylabel('Difference in Mean Index Value (With - Without Reallocation)');
xticks(1:num_scenarios);
xticklabels({'2H2R', '3H2R', '4H2R', '4H3R'});
grid on;

% Add a legend to distinguish the plot
legend;

% Display the plot
drawnow;

