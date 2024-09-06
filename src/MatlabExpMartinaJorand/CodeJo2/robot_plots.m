function robot_plots(time,q_des,q,dq_des,dq,index)
    figure
    subplot(2,1,1)
    hold on
    title(sprintf('Robot%d',index));
    plot(time,q_des(:,1),'LineWidth',2);
    plot(time,q(:,1),'LineWidth',2);
    xlabel('[s]');
    ylabel('[m]');
    xlim([0,time(end,1)]);
    legend('nominal','actual')
    grid
    
    subplot(2,1,2)
    plot(time,q_des(:,2),'LineWidth',2);
    hold on
    plot(time,q(:,2),'LineWidth',2);
    xlabel('[s]');
    ylabel('[m]');
    xlim([0,time(end,1)]);
    grid
    
    figure
    subplot(2,1,1)
    hold on
    title(sprintf('Robot%d', index));
    plot(time,dq_des(:,1),'LineWidth',2);
    plot(time,dq(:,1),'LineWidth',2);
    xlabel('[s]');
    ylabel('[m/s]');
    xlim([0,time(end,1)]);
    legend('nominal','actual')
    grid

    subplot(2,1,2)
    plot(time,dq_des(:,2),'LineWidth',2);
    hold on
    plot(time,dq(:,2),'LineWidth',2);
    xlabel('[s]');
    ylabel('[m/s]');
    xlim([0,time(end,1)]);
    grid
end