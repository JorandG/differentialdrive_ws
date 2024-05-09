function AllUpdated = updateSchedule(All, Alloc, humanTime_filling, dist, vel_min, vel_max, inv_vel_min, inv_vel_max, idx_depot_tasks, idx_going_tasks, idx_to_ignore_r, idx_to_ignore_h, agents_ordered_allocation, service_time, humanTime_fillingPrev)
    global humanTime_serving difference humanTime_fillingPrev timeReall num_tasks num_robots num_agents human num_filling_boxes num_phases 

    service_time1 = [];
    serv_time = humanTime_serving;
    waiting_time = 5;
    approaching_time = 5;
    curr_hum_slow = false;

    X = repmat(All.X,num_phases,1); 
    All1 = All;
    dist1 = dist;
    dist1 = repmat(dist1,num_phases,1);

    %Shifting human timings (to do before updating robot times)
    for curr_hum=1:num_tasks
        if All.timeFh(curr_hum)==timeReall
            %human slower
            if humanTime_filling(curr_hum) > humanTime_fillingPrev(curr_hum)
                disp('slower')
                curr_hum_slow = true;
                hum_slow = curr_hum
                checkShift = true;
                %Shifts human timings for slower human
                for s=curr_hum+num_agents:num_agents:num_tasks
                    difference
                    All.timeSh(s) = All.timeSh(s) + abs(difference) + serv_time + waiting_time + approaching_time;
                    difference = abs(All.timeFh(s) - (All.timeSh(s) + humanTime_filling(curr_hum)));
                    All.timeFh(s) = All.timeSh(s) + humanTime_filling(curr_hum);
                end
            end
            %Shifts human timings for faster human
            for s=curr_hum+num_agents:num_agents:num_tasks
                All.timeFh(s) = All.timeSh(s) + humanTime_filling(curr_hum) + serv_time + waiting_time + approaching_time;
            end
        end
    end

    for i=1:num_tasks
        curr_hum = i;
        %checkShift = false;
        %curr_hum_slow = false;
        if (ismember(curr_hum,idx_going_tasks))
            dep = curr_hum+num_tasks*(num_phases-1);
            serv = curr_hum+num_tasks*(num_phases-2);
            wait = curr_hum+num_tasks*(num_phases-3);
            app = curr_hum+num_tasks*(num_phases-4);
        end

        prev_dep_time = All.timeF(dep);

        %if All.timeF(curr_hum) < All.timeFh(curr_hum) + service_time(curr_hum) || dist1(curr_hum) ~= dist(curr_hum) && ~ismember(curr_hum, idx_to_ignore_h) %If the robot wants to serve the human before he has finished
        if ~ismember(curr_hum, idx_to_ignore_h)
            prev_timeF_curr_hum = All.timeF(curr_hum);
            if dist1(curr_hum)/(All.timeS(curr_hum)-All.timeFh(curr_hum))<max(vel_max) && curr_hum_slow %If speed up the going possible
                    disp('speed up going possible')
                    All.timeS(curr_hum) = All.timeFh(curr_hum) - nonzeros(dist1(curr_hum,:).*All.X(curr_hum,:))/max(vel_max); %We speed up to the max velocity to minimize the shifting quantity
                    All.timeF(curr_hum) = All.timeFh(curr_hum); %+ service_time1(curr_hum); %update final time of going

                    All.timeS(app)=All.timeF(curr_hum); %approaching
                    All.timeF(app)=All.timeS(app)+approaching_time;

                    All.timeS(wait)=All.timeF(app); %waiting
                    All.timeF(wait)=All.timeS(wait)+waiting_time;

                    All.timeS(serv)=All.timeF(wait); %serving
                    All.timeF(serv)=All.timeS(serv)+serv_time;%service_time1(hum_slow);

                    All.timeS(dep)=All.timeF(serv); %depot
                    All.timeF(dep)=All.timeS(dep)+nonzeros(dist1(dep,:).*X(dep,:))/max(vel_max);%update final time of depot

                %end            
            % else %Speed up not possible
            %     disp('speed up NOT possible')
            %     All.timeS(curr_hum) = All.timeFh(curr_hum) - (nonzeros(All.invprod(curr_hum,:))*nonzeros(dist1(curr_hum,:).*All.X(curr_hum,:)))
            %     All.timeF(curr_hum) = All.timeFh(curr_hum); %+ service_time1(curr_hum); %update final time of picking
            % 
            %     All.timeS(app)=All.timeF(curr_hum); %approaching
            %     All.timeF(app)=All.timeS(app)+approaching_time;
            % 
            %     All.timeS(wait)=All.timeF(app); %waiting
            %     All.timeF(wait)=All.timeS(wait)+waiting_time;
            % 
            %     All.timeS(serv)=All.timeF(wait); %serving
            %     All.timeF(serv)=All.timeS(serv)+serv_time;%service_time(dep);
            % 
            %     All.timeS(dep)=All.timeF(serv); %depot
            %     All.timeF(dep)=All.timeS(dep)+(nonzeros(All.invprod(dep,:))*nonzeros(dist1(dep,:).*X(dep,:)));%update final time of depot
            end

            variation = All.timeF(dep) - prev_dep_time

            for j=1:num_robots
                timeS = agents_ordered_allocation(j).timeS;
                timeF = agents_ordered_allocation(j).timeF;
                tasks = agents_ordered_allocation(j).tasks;
                types = agents_ordered_allocation(j).type;
                curr_num_tasks = length(tasks);
                first_task_index = find(curr_hum == tasks);
                if isempty(first_task_index)
                    % the robot is not executing curr_hum
                    % task
                    first_task_index = find(timeS >= prev_timeF_curr_hum, 1, 'first');
                    if ~isempty(first_task_index)
                        if strcmp(types{first_task_index}, 'depot')
                            first_task_index = first_task_index + 1;
                        else
                            % we go to the next picking
                            first_task_index = first_task_index + num_phases;
                        end
                    end
                else
                    % the robot is doing the task curr_hum
                    % and we skip to the next pick
                    agents_ordered_allocation(j).timeS(first_task_index) = All.timeS(curr_hum);
                    agents_ordered_allocation(j).timeF(first_task_index) = All.timeF(curr_hum);
                    first_task_index = first_task_index + num_phases;
                end

                for l=first_task_index:curr_num_tasks
                    if All.timeF(tasks(l-1)) > All1.timeF(tasks(l-1))
                        All.timeS(tasks(l)) = All.timeS(tasks(l))+variation;
                        All.timeF(tasks(l)) = All.timeF(tasks(l))+variation;
                        agents_ordered_allocation(j).timeS(l) = All.timeS(tasks(l));
                        agents_ordered_allocation(j).timeF(l) = All.timeF(tasks(l));
                    end
                end
            end
        elseif ismember(curr_hum, idx_to_ignore_h) %&& curr_hum_slow
            disp('shifting current human')
            deltar = All.timeFh(curr_hum) - All.timeF(curr_hum);
            if ~curr_hum_slow
                All.timeS(curr_hum) = All1.timeS(curr_hum);
                All.timeF(curr_hum) = All1.timeF(curr_hum);
                All.timeS(app) = All.timeF(curr_hum)
                All.timeF(app) = All.timeS(app) + approaching_time
                All.timeS(wait) = All.timeF(app); %All.timeS(wait) + deltar; %waiting
                All.timeF(wait) = All.timeS(wait) + waiting_time; %All.timeF(wait) + deltar;
    
                All.timeS(serv) = All.timeF(wait); %serving
                All.timeF(serv) = All.timeS(serv) + serv_time(curr_hum);%service_time1(hum_slow);
    
                All.timeS(dep) = All.timeF(serv);%All.timeS(dep) + deltar; %depot
                All.timeF(dep) = All.timeS(dep) + nonzeros(dist1(dep,:).*X(dep,:))/max(vel_max);%update final time of depot
            else
                All.timeF(curr_hum) = All.timeFh(curr_hum); %+ service_time1(hum_slow); %update final time of going
                All.timeS(app) = All.timeS(app) + deltar; %approaching
                All.timeF(app) = All.timeF(app) + deltar;
    
                All.timeS(wait) = All.timeF(app); %All.timeS(wait) + deltar; %waiting
                All.timeF(wait) = All.timeS(wait) + waiting_time; %All.timeF(wait) + deltar;
    
                All.timeS(serv) = All.timeF(wait); %serving
                All.timeF(serv) = All.timeS(serv) + serv_time(curr_hum);%service_time1(hum_slow);
    
                All.timeS(dep) = All.timeF(serv);%All.timeS(dep) + deltar; %depot
                All.timeF(dep) = All.timeF(dep) + deltar;%update final time of depot
            end
        end

    end

    All.makespan = max(All.timeF);
    AllUpdated = All;
end