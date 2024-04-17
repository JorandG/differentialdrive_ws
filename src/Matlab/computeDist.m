function [distance, service_t] = computeDist(num_tasks, num_robots, position_human, ptasks, num_service_tasks, probot, idx_going_tasks, service_time)

    dist = zeros(num_tasks, num_robots);
    for i = 1:num_tasks
        for j = 1:num_robots
            if find(i == idx_going_tasks)
                dist(i,j) = norm(probot(:,j)-ptasks(:,i));
                service_time(i,j) = service_time(i,j) + 15;
            else
                dist(i,j) = norm(probot(:,j) - ptasks(:,i-num_service_tasks));
                service_time(i,j) = 5;
            end
        end
    end
    distance = dist;
    service_t = service_time;
end