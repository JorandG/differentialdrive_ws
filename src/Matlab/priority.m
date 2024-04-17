function priority()
    global humanData TimeHumFilling1 Ph num_phases dist num_agents human num_robots tasknum tasknum1 humanTime_filling num_filling_boxes num_service_tasks vel_min vel_max inv_vel_max inv_vel_min M idx_to_consider_h idx_to_ignore_h idx_to_consider_r idx_to_ignore_r first_allocation
    TimeHumFilling1
    for i=1:num_agents
        Ph(i) = human{i}.TimeHumFilling(i)/150 + humanData{i}.RobotVelocity;
    end
end