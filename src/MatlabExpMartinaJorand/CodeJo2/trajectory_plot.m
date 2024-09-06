function trajectory_plot(waypoints,q,q_des)
    figure
    title('Planned L-shaped Trajectory');
    grid on;
    hold on;
    x_values = [];
    y_values = [];

    for i=1:length(waypoints)
        x_values(end+1,:) = waypoints{i}(:,1)';
        y_values(end+1,:) = waypoints{i}(:,2)';
    end
    
    xlim([min(x_values,[],'all') - 1, max(x_values,[],'all') + 1]);
    ylim([min(y_values,[],'all') - 1, max(y_values,[],'all') + 1]);
    
    for i=1:length(waypoints)
        plot(waypoints{i}(:,1)',waypoints{i}(:,2)', '-o', 'LineWidth', 2);
        if length(q) == 0 && length(q_des) > 0
            plot(q_des{i}(:,1),q_des{i}(:,2), '--', 'LineWidth', 2, 'Color', "#77AC30");
        end

        if length(q) > 0 && length(q_des) == 0
            plot(q{i}(:,1),q{i}(:,2), '-.', 'LineWidth', 2, 'Color', "#D95319");
        end
    end
end