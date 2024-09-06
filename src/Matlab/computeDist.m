% function [distance, service_t] = computeDist(num_tasks, num_robots, position_human, ptasks, num_service_tasks, probot, idx_going_tasks, service_time)
% 
%     dist = zeros(num_tasks, num_robots);
%     for i = 1:num_tasks
%         for j = 1:num_robots
%             if find(i == idx_going_tasks)
%                 dist(i,j) = norm(probot(:,j)-ptasks(:,i));
%                 service_time(i,j) = service_time(i,j) + 15;
%             else
%                 dist(i,j) = norm(probot(:,j) - ptasks(:,i-num_service_tasks));
%                 service_time(i,j) = 5;
%             end
%         end
%     end
%     distance = dist;
%     service_t = service_time;
% end

function [distance, service_t] = computeDist(num_tasks, num_robots, position_human, ptasks, num_service_tasks, probot, idx_going_tasks, service_time)

    dist = zeros(num_tasks, num_robots);  % Initialize distance matrix
    for i = 1:num_tasks
        for j = 1:num_robots
            if find(i == idx_going_tasks)
                % Access the specific x and y coordinates for probot and ptasks
                dist(i,j) = abs(probot(j,1) - ptasks(1,i)) + abs(probot(j,2) - ptasks(2,i));  % Manhattan distance
                service_time(i,j) = service_time(i,j) + 15;
            else
                dist(i,j) = abs(probot(j,1) - ptasks(1,i-num_service_tasks)) + abs(probot(j,2) - ptasks(2,i-num_service_tasks));
                service_time(i,j) = 7;
            end
        end
    end
    distance = dist;
    service_t = service_time;
end
