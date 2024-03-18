addpath('/opt/gurobi1000/linux64/matlab');
savepath;


global Ph human num_agents num_humans num_farming_robot tasknum tasknum1 num_robots initialTime num_filling_boxes
global Ph idx_going_tasks idx_depot_tasks idx_waiting_tasks idx_services_tasks dist num_robots num_humans num_farming_robot num_filling_boxes VelRob HumWait HumTimeList num_service_tasks num_tasks num_depot_tasks service_time num_agents human humanTime_filling humanTime_filling1 WeightHumanwaiting WeightEnergyPicking WeightMakespan vel_min vel_max inv_vel_max inv_vel_min M idx_to_consider_h idx_to_ignore_h idx_to_consider_r idx_to_ignore_r first_allocation

num_humans = 2;
num_agents = num_humans + num_farming_robot
initialTime = 0;

for i=1:num_agents
    for j=1:num_filling_boxes
        disp('init')
        human{i}.TimeFinishFilling = [];
        human{i}.TimeStartFilling = [];
        human{i}.TimeStartServing = [];
        human{i}.TimeFinishServing = [];
        human{i}.TimeHumFilling =  [];
        
    end
end

for i=1:num_agents
    Ph(i) = 1;
    human{i}.HumanIDEditField.Value = i;
    human{i}.alpha = 1;
    for j=1:num_filling_boxes
        disp('init')
        human{i}.TimeFinishFilling(end+1) = initialTime;
        human{i}.TimeStartFilling(end+1) = 1;
        human{i}.TimeStartServing(end+1) = 0;
        human{i}.TimeFinishServing(end+1) = 15;
        human{i}.TimeHumFilling(end+1) = initialTime;
        human{i}.NumTask = 1; 
        human{i}.StartFilling = 1;
        human{i}.FinishFilling = 0;
        human{i}.StartServing = 0;
        human{i}.FinishServing = 0;
    end
end
for j=1:num_robots
    tasknum{j} = 1;
    tasknum1{j} = 1;
end


rng(1)  
num_agents = num_farming_robot + num_humans;

num_service_tasks = num_filling_boxes*num_agents;
num_depot_tasks = num_service_tasks;
num_tasks = num_service_tasks;
service_time = zeros(num_tasks, num_robots);

M = 10000;
num_agents = num_humans + num_farming_robot
vel_min = ones(num_robots,1)*0.2; % min velocity for the robots
vel_max = ones(num_robots,1)*0.4; % max velocity for the robots
tolerance = 1e-5;
% Weights for objective function
WeightHumanwaiting = 1;
WeightEnergyPicking = 1;
WeightMakespan = 1;
std_dev = 140;
inv_vel_min = 1./vel_min;
inv_vel_max = 1./vel_max;

idx_going_tasks = 1:num_service_tasks;
idx_waiting_tasks = num_service_tasks+1:num_service_tasks*2;
idx_services_tasks = num_service_tasks*2+1:num_service_tasks*3;
idx_depot_tasks = num_service_tasks*3+1:num_service_tasks*4;
% Positions
min_base_distance = 5.5;
%probot = [zeros(1,num_robots); 0:min_base_distance:min_base_distance*(num_robots-1)]
probot = [2, 3, 4, 5; 1, 1, 1, 1];
i_values = [4, 7, 8, 11, 12, 14];% x positions of the humans in the field
j_values = 5:10;% y positions of the humans
position_hum = zeros(num_agents,2);
for i=1:num_agents
    indi = randi(length(i_values), 1);
    indj = randi(length(j_values), 1);
    position_hum(i,1) = i_values(indi);
    position_hum(i,2) = j_values(indj);
end
ptasks = repmat(position_hum.', 1, num_filling_boxes);
[dist, service_time] = computeDist(num_tasks, num_robots, position_hum, ptasks, num_service_tasks, probot, idx_going_tasks, service_time);
timeReall = 0;
idx_to_consider_h = [];
idx_to_consider_r = [];
idx_to_ignore_h = [];
idx_to_ignore_r = [];
first_allocation = 0;
UpdatedHumanTime_filling = 0;
ReAll = 0;
checkConstraints = 1;
humanTime_filling1 = ones(1, num_agents)*150;
humanTime_filling = ones(1, num_agents*num_filling_boxes)*150; %Find the first paramerters randomly for the allocation
%humanTime_filling = [491 253 346];
humanTime_filling_ext = repmat(humanTime_filling1, 1, num_filling_boxes);
drawnow
ReAll = Reallocation(num_service_tasks, num_tasks, num_agents, num_filling_boxes, num_robots, service_time, timeReall, humanTime_filling, ReAll);
%HumTimeList = zeros(1, num_agents);
display(ReAll, num_robots, num_agents, num_filling_boxes, idx_going_tasks, timeReall);
simulation(ReAll, idx_going_tasks, dist, vel_min, vel_max, inv_vel_min, inv_vel_max, idx_depot_tasks, service_time, num_tasks, idx_to_consider_r, idx_to_consider_h, idx_to_ignore_r, idx_to_ignore_h)



function Reall = Reallocation(num_service_tasks, num_tasks, num_agents, num_filling_boxes, num_robots, service_time, timeReall, Time_filling, ReAll)
    global Ph idx_going_tasks idx_depot_tasks idx_waiting_tasks idx_services_tasks dist HumMaxVelRobot num_humans num_farming_robot num_agents human humanTime_filling1 WeightHumanwaiting WeightEnergyPicking WeightMakespan vel_min vel_max inv_vel_max inv_vel_min M idx_to_consider_h idx_to_ignore_h idx_to_consider_r idx_to_ignore_r first_allocation num_phases

    waiting_time = 5;
    serv_time = 7;
    num_phases = 4;
    for k=1:num_agents
        app.alphaWeight(k) = human{k}.alpha;
    end
    app.alphaWeight;
    HumMaxVelRobot1 = 1./HumMaxVelRobot';
    inversedistance = 0;
    inversedistancegoing = 0;
    inversedistancedepot= 0;
    tolerance = 1e-5;
    num_depot_tasks = num_service_tasks;
    idx_going_tasks = 1:num_service_tasks;
    idx_waiting_tasks = num_service_tasks+1:num_service_tasks*2;
    idx_services_tasks = num_service_tasks*2+1:num_service_tasks*3;
    idx_depot_tasks = num_service_tasks*3+1:num_service_tasks*4;
    humanTime_filling = Time_filling; %Find the first paramerters randomly for the allocation
    humanTime_filling_ext = repmat(humanTime_filling1, 1, num_filling_boxes);
    humanwaiting = 0.0;

    %% Optimization variables

    X = optimvar('X', [num_tasks, num_robots],'Type', 'integer', 'LowerBound',0,'UpperBound',1 );
    Paux = optimvar('Paux', [(sum(1:num_tasks*num_phases)-1)*num_robots],'Type', 'integer', 'LowerBound',0,'UpperBound',1 );
    timeS = optimvar('timeS', [1 num_tasks*num_phases] ,'LowerBound', 0);
    timeF = optimvar('timeF', [1 num_tasks*num_phases] ,'LowerBound', 0);
    invprod = optimvar('invprod', [num_tasks*num_phases, num_robots] ,'LowerBound', 0);
    veloc = optimvar('veloc', [num_tasks*num_phases, num_robots] ,'LowerBound', 0);
    timeSh = optimvar('timeSh', [1 num_service_tasks] ,'LowerBound', 0);
    timeFh = optimvar('timeFh', [1 num_service_tasks] ,'LowerBound', 0);
    makespan = optimvar('makespan','LowerBound',0);%overall duration
    velocitypicking = optimvar('velocitypicking','LowerBound',0);%velocity of robot for picking
    v_min = min(vel_min);
    v_max = max(vel_max);
    normavel = (v_min*v_max)/((v_max-v_min)*num_tasks);
    normawait = 1/(num_agents*num_filling_boxes*(max(dist, [], 'all')/v_min));
    normamakespan = (makespan/(num_service_tasks*(2*max(dist, [], 'all')/v_min)+max(max(service_time(1:num_agents*num_filling_boxes,:))+max(max(service_time(1:num_agents*num_filling_boxes,:))))));

    for i=1:num_agents
        humanwaiting1 = Ph(i)*(sum(timeSh(num_agents+i:num_agents:num_service_tasks) - timeFh(i:num_agents:num_service_tasks-num_agents) + timeSh(i))); %app.alphaWeight(i)*
        humanwaiting = humanwaiting + humanwaiting1;
    end
    %% Problem definition
    prob = optimproblem;
    prob.Objective =  WeightHumanwaiting*(humanwaiting)+WeightEnergyPicking*velocitypicking+WeightMakespan*normamakespan; %+WeightEnergyDepositing*(num_tasks*(1/min(vel_min))-sum(invprod,'all'));

    %% Constraints
    prob.Constraints.makespanbound =  makespan >= timeF;
    prob.Constraints.velocitypickingduration1 = velocitypicking == (num_tasks*(1/min(vel_min))-sum(invprod,'all'))*normavel;

    idx_tasks = [idx_going_tasks, idx_waiting_tasks, idx_services_tasks, idx_depot_tasks];
    X1 = repmat(X,num_phases,1);

    if first_allocation == 2
        X2 = repmat(ReAll.X,num_phases,1);
        for u=1:num_agents
            count = u;
            for v=1:num_filling_boxes
                humanTime_filling_ext(count) = human{u}.TimeFinishFilling(v);
                %service_time(count) = human{u}.TimeFinishServing(v);
                count = count + num_agents;
            end
        end
        idx_going_tasks_consider = idx_to_consider_r;
        idx_waiting_tasks_consider = idx_going_tasks_consider + num_service_tasks;
        idx_services_tasks_consider = idx_waiting_tasks_consider + num_service_tasks;
        idx_depot_tasks_consider = idx_services_tasks_consider + num_service_tasks;
        idx_to_consider_r = [idx_going_tasks_consider, idx_waiting_tasks_consider, idx_services_tasks_consider, idx_depot_tasks_consider];
        
        idx_going_tasks_ignore = idx_to_ignore_r;
        idx_waiting_tasks_ignore = idx_going_tasks_ignore + num_service_tasks;
        idx_services_tasks_ignore = idx_waiting_tasks_ignore + num_service_tasks;
        idx_depot_tasks_ignore = idx_services_tasks_ignore + num_service_tasks;
        idx_to_ignore_r = [idx_going_tasks_ignore, idx_waiting_tasks_ignore, idx_services_tasks_ignore, idx_depot_tasks_ignore];
        
        %% Tasks duration
        dist = repmat(dist,num_phases,1);
        invdist = invprod(idx_tasks,:).*dist(idx_tasks,:);
        for r=1:num_robots
            inversedistance = inversedistance + invdist(:,r);
        end
        inverse_dist = repmat(inversedistance, 1, num_robots);

        %Timings for going and depositing
        prob.Constraints.duration1 = (repmat(timeF(idx_depot_tasks_consider), num_robots,1)'-repmat(timeS(idx_depot_tasks_consider), num_robots,1)')==inverse_dist(idx_depot_tasks_consider,:);
        prob.Constraints.duration2 = (repmat(timeF(idx_going_tasks_consider), num_robots,1)'-repmat(timeS(idx_going_tasks_consider), num_robots,1)')==inverse_dist(idx_going_tasks_consider,:);

        prob.Constraints.duration3 = invprod(idx_going_tasks_consider,:) >= X1(idx_going_tasks_consider,:).*repmat((inv_vel_max)', length(idx_going_tasks_consider),1);
        prob.Constraints.duration4 = invprod(idx_going_tasks_consider,:) <= X1(idx_going_tasks_consider,:).*repmat((inv_vel_min)', length(idx_going_tasks_consider),1);
        prob.Constraints.duration5 = invprod(idx_depot_tasks_consider,:) >= X1(idx_depot_tasks_consider,:).*repmat((inv_vel_max)', length(idx_going_tasks_consider),1);
        prob.Constraints.duration6 = invprod(idx_depot_tasks_consider,:) <= X1(idx_depot_tasks_consider,:).*repmat((inv_vel_min)', length(idx_going_tasks_consider),1);

        prob.Constraints.duration8 = invprod(idx_to_ignore_r,:) == ReAll.invprod(idx_to_ignore_r,:);

        prob.Constraints.timeF_tasks_to_ignore = timeF(idx_to_ignore_r) == ReAll.timeF(idx_to_ignore_r);
        prob.Constraints.timeS_tasks_to_ignore = timeS(idx_to_ignore_r) == ReAll.timeS(idx_to_ignore_r);
        prob.Constraints.X_tasks_to_ignore = X1(idx_to_ignore_r,:) == X2(idx_to_ignore_r,:);


        prob.Constraints.waiting_time = timeS(idx_waiting_tasks_consider) == timeF(idx_going_tasks_consider); %waiting for human
        prob.Constraints.waiting_time1 = timeF(idx_waiting_tasks_consider) == timeS(idx_waiting_tasks_consider) + 5;%(timeSh(num_humans+1:length(idx_waiting_tasks_consider))-timeF(idx_services_tasks_consider(1:length(idx_waiting_tasks_consider)-num_humans)))/(max(dist(:))/min(vel_min) + serv_time)
        
        prob.Constraints.services_time = timeS(idx_services_tasks_consider) == timeF(idx_waiting_tasks_consider); %time to make the service to the human
        prob.Constraints.services_time2 = timeF(idx_services_tasks_consider) == timeS(idx_services_tasks_consider) + serv_time; %time to make the service to the human
        prob.Constraints.depot_time = timeS(idx_depot_tasks_consider) == timeF(idx_services_tasks_consider);

        prob.Constraints.humanduration1 = timeFh(idx_to_ignore_h) == ReAll.timeFh(idx_to_ignore_h);
        prob.Constraints.humanduration3 = timeSh(idx_to_ignore_h) == ReAll.timeSh(idx_to_ignore_h);
        prob.Constraints.humanduration4 = timeFh(idx_to_consider_h) == timeSh(idx_to_consider_h) + humanTime_filling_ext(idx_to_consider_h);

    elseif first_allocation == 0
        % Tasks duration
        invdist = invprod.*repmat(dist,num_phases,1);
        for r=1:num_robots
            inversedistance = inversedistance + invdist(:,r);
        end
        inverse_dist = repmat(inversedistance, 1, num_robots);
        %Timings for going and depositing
        prob.Constraints.duration1 = (repmat(timeF(idx_depot_tasks), num_robots,1)'-repmat(timeS(idx_depot_tasks), num_robots,1)')==inverse_dist(idx_depot_tasks,:);
        prob.Constraints.duration2 = (repmat(timeF(idx_going_tasks), num_robots,1)'-repmat(timeS(idx_going_tasks), num_robots,1)')==inverse_dist(idx_going_tasks,:);

        prob.Constraints.duration3 = invprod(idx_going_tasks,:) >= X1(idx_going_tasks,:).*repmat((inv_vel_max)', num_tasks,1);
        prob.Constraints.duration4 = invprod(idx_going_tasks,:) <= X1(idx_going_tasks,:).*repmat((inv_vel_min)', num_tasks,1);
        prob.Constraints.duration5 = invprod(idx_depot_tasks,:) >= X1(idx_depot_tasks,:).*repmat((inv_vel_max)', num_tasks,1);
        prob.Constraints.duration6 = invprod(idx_depot_tasks,:) <= X1(idx_depot_tasks,:).*repmat((inv_vel_min)', num_tasks,1);

        prob.Constraints.waiting_time01 = timeF(idx_waiting_tasks(1:num_humans)) == timeF(idx_going_tasks(1:num_humans)) + 5;
        prob.Constraints.waiting_time02 = timeS(idx_waiting_tasks(1:num_humans)) == timeF(idx_going_tasks(1:num_humans));
        prob.Constraints.waiting_time = timeS(idx_waiting_tasks(num_humans+1:length(idx_waiting_tasks))) == timeF(idx_going_tasks(num_humans+1:length(idx_waiting_tasks))); %waiting for human
        prob.Constraints.waiting_time1 = timeF(idx_waiting_tasks(num_humans+1:length(idx_waiting_tasks))) == timeS(idx_waiting_tasks(num_humans+1:length(idx_waiting_tasks))) + 5;%(timeSh(num_humans+1:length(idx_waiting_tasks))-timeF(idx_services_tasks(1:length(idx_waiting_tasks)-num_humans)))/(max(dist(:))/min(vel_min) + serv_time)
        
        prob.Constraints.services_time = timeS(idx_services_tasks) == timeF(idx_waiting_tasks); %time to make the service to the human
        prob.Constraints.services_time2 = timeF(idx_services_tasks) == timeS(idx_services_tasks) + serv_time; %time to make the service to the human
        prob.Constraints.depot_time = timeS(idx_depot_tasks) == timeF(idx_services_tasks);

        prob.Constraints.humanduration = timeFh == timeSh + humanTime_filling_ext;
    end

    % Each service task must be assigned to exactly one agents.
    assigneachtask = sum(X(idx_going_tasks,:),2) == 1;
    prob.Constraints.assigneachtask = assigneachtask;
    % Precedence for human filling tasks
    human_start = optimconstr(num_agents*(num_filling_boxes-1));
    curr_idx = 1;
    for i = 1:num_agents
        idx_curr_hum = i:num_agents:num_agents*num_filling_boxes;
        human_start(curr_idx:curr_idx+num_filling_boxes-2) = timeSh(idx_curr_hum(2:end)) >= timeFh(idx_curr_hum(1:end-1)); %const 5
        curr_idx = curr_idx + num_filling_boxes -1;
    end
    prob.Constraints.human_start = human_start;

    robot_service = optimconstr(num_agents*num_filling_boxes);
    curr_idx = 1;
    for i = 1:num_agents
        idx_curr_hum = i:num_agents:num_agents*num_filling_boxes;
        robot_service(curr_idx:curr_idx+num_filling_boxes-1) = timeF(idx_going_tasks(idx_curr_hum)) >= timeFh(idx_curr_hum); %+service_time(idx_curr_hum);
        curr_idx = curr_idx +num_filling_boxes;
    end
    prob.Constraints.robot_service = robot_service;

    % a human cannot start a task if the box has not been removed
    human_waiting = optimconstr(num_agents*(num_filling_boxes-1));
    curr_idx = 1;
    for i = 1:num_agents
        idx_curr_hum = i:num_agents:num_agents*num_filling_boxes;
        human_waiting(curr_idx:curr_idx+num_filling_boxes-2) = timeSh(idx_curr_hum(2:end)) >= timeF(idx_services_tasks(idx_curr_hum(1:end-1))); %const 6
        curr_idx = curr_idx +num_filling_boxes-1;
    end
    prob.Constraints.human_waiting = human_waiting;

    % No simultaneous tasks
    parallelism = optimconstr(2*(sum(1:num_tasks*num_phases)-1)*num_robots);
    r=1;
    l = 1;
    for k=1:num_robots
        for i=1:num_tasks
            for j=(i+1):num_tasks*num_phases
                parallelism(r) = timeS(i)-timeF(j)>=-M*(2-X(i,k)-X1(j,k))-M*Paux(l); r=r+1;
                parallelism(r) = timeS(j)-timeF(i)>=-M*(2-X(i,k)-X1(j,k))-M*(1-Paux(l));r=r+1;
                l=l+1;
            end
        end
    end
    prob.Constraints.parallelism = parallelism;

    options = optimoptions('intlinprog','Display',"iter");
    [sol,fval,exitflag] = solve(prob,'Options',options, 'Solver', 'intlinprog');
    disp(['Cost: ', num2str(max(sol.timeF)) ]);
    Reall = sol, humanTime_filling;

end

function AllUpdated = updateSchedule(All, Alloc, humanTime_filling, dist, vel_min, vel_max, inv_vel_min, inv_vel_max, idx_depot_tasks, idx_going_tasks, idx_to_ignore_r, idx_to_ignore_h, agents_ordered_allocation, service_time)
    global  num_robots num_agents human num_filling_boxes num_phases 

    humanTime_fillingUpdated = [];
    service_time1 = [];
    serv_time = 7;
    waiting_time = 0;
    X = repmat(All.X,num_phases,1); 
    
    for y=1:num_agents*num_filling_boxes
        humanTime_fillingUpdated(end+1) = 1;
        service_time1(end+1) = 1;
    end
    for u=1:num_agents
        count = u;
        for v=1:num_filling_boxes
            humanTime_fillingUpdated(count) = human{u}.TimeFinishFilling(v);
            service_time1(count) = human{u}.TimeFinishServing(v);
            count = count + num_agents;
        end
    end

    All1 = All
    dist1 = dist
    dist1 = repmat(dist1,num_phases,1);
    for i=1:num_agents*num_filling_boxes
        curr_hum = i;
        checkShift = false;
        curr_hum_slow = false;
        if (ismember(curr_hum,idx_going_tasks))
            dep = curr_hum+num_agents*num_filling_boxes*(num_phases-1);
            serv = curr_hum+num_agents*num_filling_boxes*(num_phases-2);
            wait = curr_hum+num_agents*num_filling_boxes*(num_phases-3);
        end
        %%%%%%%%%%%%%%% service_time
        if service_time1(curr_hum) ~= service_time(curr_hum) && ~ismember(curr_hum, idx_to_ignore_h)
            diff_service_time = service_time1(curr_hum) - service_time(curr_hum);
            All.timeFh(curr_hum) = All.timeFh(curr_hum) + diff_service_time;
            All.timeS(dep) = All.timeF(curr_hum) + service_time1(curr_hum);
            All.timeF(dep) = All.timeF(dep) + diff_service_time;
            All.timeF(serv) = All.timeF(serv) + diff_service_time;
            All.timeF(wait) = All.timeF(wait) + diff_service_time;
        end
        %%%%%%%%%%%%%%%%%

        %%%%%%%%%%%%%%% Human time filling
        %if humanTime_fillingUpdated(curr_hum) ~= humanTime_filling(curr_hum) && ~ismember(curr_hum, idx_to_ignore_h) %if the new time needed to fill a box is different from before and is not a task to ignore
            All.timeFh(curr_hum) = All.timeSh(curr_hum) + humanTime_fillingUpdated(curr_hum); %We update the finishing time
            curr_hum_slow = true;
            hum_slow = curr_hum;
            checkShift = true;
            if (curr_hum + num_agents)<num_agents*num_filling_boxes+1 && Alloc.timeSh(curr_hum + num_agents) < All.timeFh(curr_hum) %If the starting time of the next human task is lower than the finishing time of the current human task i.e. the human is slower
                disp('slower')
                All.timeSh(curr_hum + num_agents) = All.timeFh(curr_hum) + serv_time;%service_time1(curr_hum); %We update the starting time of the next task
            elseif (curr_hum + num_agents)<num_agents*num_filling_boxes+1 && Alloc.timeSh(curr_hum + num_agents) > All.timeFh(curr_hum) %If the starting time of the next human task is higher than the finishing time of the current human task i.e. the human is faster
                disp('faster')
                All.timeSh(curr_hum+num_agents) = All.timeFh(curr_hum) + serv_time;%service_time1(curr_hum);%We update the starting time of the next task
            end
        %end
    
        %%%%%%%%%%%%%%%%%%
        prev_dep_time = All.timeF(dep);
        %%%%%%%%%%%%%%%%%%% Distance
%                 if dist1(curr_hum) ~= dist(curr_hum) && ~ismember(curr_hum, idx_to_ignore_h) %if the new time distance is different from before and is not a task to ignore
%                     All.timeF(curr_hum)=All.timeS(curr_hum)+(nonzeros(All.invprod(curr_hum,:)*nonzeros(dist1(curr_hum,:).*All.X(curr_hum,:))));
%                     All.timeS(dep)=All.timeF(curr_hum) + service_time1(hum_slow); %depot
%                     All.timeF(dep)=All.timeS(dep)+(nonzeros(All.invprod(dep,:)*nonzeros(dist1(dep,:).*All.X(dep,:))));
%                 end
        %%%%%%%%%%%%%%%%%%%%


        if All.timeF(curr_hum) < All.timeFh(curr_hum) + service_time(curr_hum) || dist1(curr_hum) ~= dist(curr_hum) && ~ismember(curr_hum, idx_to_ignore_h) %If the robot wants to serve the human before he has finished
            prev_timeF_curr_hum = All.timeF(curr_hum);
            if dist1/(All.timeS(curr_hum)-All.timeFh(curr_hum))<max(vel_max) %If speed up the going possible
                if curr_hum_slow
                    disp('speed up going possible')
                    All.timeS(hum_slow) = All.timeFh(hum_slow) - nonzeros(dist1(hum_slow,:).*All.X(hum_slow,:))/max(vel_max); %We speed up to the max velocity to minimize the shifting quantity
                    All.timeF(hum_slow) = All.timeFh(hum_slow); %+ service_time1(hum_slow); %update final time of going
                    %All.timeS(dep)=All.timeF(hum_slow) + service_time1(hum_slow); %depot
                    %All.timeF(dep)=All.timeS(dep)+max(vel_max)*nonzeros(dist1(dep,:).*All.X(dep,:)); %+ service_time(dep);%update final time of depot no need to use service_time1

                    All.timeS(wait)=All.timeF(hum_slow); %waiting
                    All.timeF(wait)=All.timeS(wait)+waiting_time;
                    All.timeS(serv)=All.timeF(wait); %serving
                    All.timeF(serv)=All.timeS(serv)+serv_time;%service_time1(hum_slow);
                    All.timeS(dep)=All.timeF(serv); %depot
                    All.timeF(dep)=All.timeS(dep)+nonzeros(dist1(dep,:).*X(dep,:))/max(vel_max);%update final time of depot
                end
            else %Speed up not possible
                disp('speed up NOT possible')
                All.timeS(curr_hum) = All.timeFh(curr_hum) - (nonzeros(All.invprod(curr_hum,:))*nonzeros(dist1(curr_hum,:).*All.X(curr_hum,:)))
                All.timeF(curr_hum) = All.timeFh(curr_hum); %+ service_time1(curr_hum); %update final time of picking
                %All.timeS(dep)=All.timeF(curr_hum) + service_time1(curr_hum); %depot
                %All.timeF(dep)=All.timeS(dep)+(nonzeros(All.invprod(dep,:))*nonzeros(dist1(dep,:).*All.X(dep,:))) + service_time(dep);%update final time of depot

                All.timeS(wait)=All.timeF(curr_hum); %waiting
                All.timeF(wait)=All.timeS(wait)+waiting_time;
                All.timeS(serv)=All.timeF(wait); %serving
                All.timeF(serv)=All.timeS(serv)+serv_time;%service_time(dep);
                All.timeS(dep)=All.timeF(serv); %depot
                All.timeF(dep)=All.timeS(dep)+(nonzeros(All.invprod(dep,:))*nonzeros(dist1(dep,:).*X(dep,:)));%update final time of depot
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
                            first_task_index = first_task_index + 4;
                        end
                    end

                else
                    % the robot is doing the task curr_hum
                    % and we skip to the next pick
                    agents_ordered_allocation(j).timeS(first_task_index) = All.timeS(curr_hum);
                    agents_ordered_allocation(j).timeF(first_task_index) = All.timeF(curr_hum);
                    first_task_index = first_task_index + 4;
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
        end
    end
    for j = 1:num_agents
        idx_curr_hum = j:num_agents:num_agents*num_filling_boxes;
        for i = 2:num_filling_boxes
            if All.timeSh(idx_curr_hum(i)) < All.timeF(idx_curr_hum(i-1))
                curr_duration = All.timeFh(idx_curr_hum(i)) - All.timeSh(idx_curr_hum(i));
                All.timeSh(idx_curr_hum(i)) = All.timeF(idx_curr_hum(i-1));
                All.timeFh(idx_curr_hum(i)) = All.timeSh(idx_curr_hum(i)) + curr_duration;
            end
        end
    end
    All.makespan = max(All.timeF);
    AllUpdated = All;
end

function display(ReAll, num_robots, num_agents, num_filling_boxes, idx_going_tasks, timeReAll)
    global num_phases
    %% Display
    close all
    rng(28)
    X1 = repmat(ReAll.X,num_phases,1);
    colors_matrix = rand(num_agents*num_filling_boxes*4,3);
    figure;
    xaxisproperties= get(gca, 'XAxis');
    xaxisproperties.TickLabelInterpreter = 'latex';
    yaxisproperties= get(gca, 'YAxis');
    yaxisproperties.TickLabelInterpreter = 'latex'; % tex for y-axis
    fontsize = 12;

    hold on

    for i=1:num_robots
        index = find(X1(:,i)>0.1);
        inittime = ReAll.timeS(X1(:,i)>0.1);
        endtime = ReAll.timeF(X1(:,i)>0.1);
        for k=1:length(inittime)
            displacement = 0.07;
            if find(index(k) == idx_going_tasks)
                h_idx = mod(index(k), num_agents);
                if h_idx == 0
                    h_idx = num_agents;
                end
                t_idx = ceil(index(k)/num_agents);
                plot([i i], [inittime(k) endtime(k)],'-', 'LineWidth', 5, 'Color', colors_matrix(h_idx*(num_filling_boxes*num_agents-1)+t_idx,:))
                processText = ['$\tau_{', num2str(h_idx),',', num2str(t_idx),  '}^p$'];
            else
                displacement = -0.35;
                idxdep = index(k)-max(idx_going_tasks);
                h_idx = mod(idxdep, num_agents);
                if h_idx == 0
                    h_idx = num_agents;
                end
                t_idx = ceil(idxdep/num_agents);
                plot([i i], [inittime(k) endtime(k)],'-', 'LineWidth', 2, 'Color', colors_matrix(h_idx*(num_filling_boxes*num_agents-1)+t_idx,:));
                processText = ['$\tau_{', num2str(h_idx),',', num2str(t_idx),  '}^d$'];
            end
            text(i+displacement,mean([inittime(k) endtime(k)]),processText,'Interpreter','latex', 'FontSize',fontsize);
        end
    end

    for i=1:num_agents
        idx_curr_hum = i:num_agents:num_agents*num_filling_boxes;
        inittime = ReAll.timeSh(idx_curr_hum);
        endtime = ReAll.timeFh(idx_curr_hum);
        for k=1:length(inittime)
            plot([i+num_robots i+num_robots], [inittime(k) endtime(k)], 'LineWidth', 5, 'Color', colors_matrix(i*(num_filling_boxes*num_agents-1)+k,:));

            processText = ['$\tau_{', num2str(i),',', num2str(k), '}^o$'];
            text(i+num_robots+0.1,mean([inittime(k) endtime(k)]),processText,'Interpreter','latex','FontSize',fontsize);
        end
    end
    xlim([0 num_robots+num_agents + 1]);
    xlabel('Agents','Interpreter','latex','FontSize',fontsize);
    ylabel('t[s]','Interpreter','latex','FontSize',fontsize);

    labels{1} = '';
    for i=1:num_robots
        labels{i+1} = ['$r_{', num2str(i), '}$'];
    end
    for i=1:num_agents
        labels{num_robots+i+1} = ['$h_{', num2str(i), '}$'];
    end

    set(gca, 'XTick', [0:num_agents+num_robots], 'XTickLabel', labels, 'FontSize',fontsize);
    grid
    box on
end

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

function priority()
    global TimeHumFilling1 Ph num_phases dist num_agents human num_robots tasknum tasknum1 humanTime_filling num_filling_boxes num_service_tasks vel_min vel_max inv_vel_max inv_vel_min M idx_to_consider_h idx_to_ignore_h idx_to_consider_r idx_to_ignore_r first_allocation
    TimeHumFilling1
    for i=1:num_agents
        Ph(i) = human{i}.TimeHumFilling(i)/150 + human{i}.VelocityRobot;
    end
end


function simulation(ReAll, idx_going_tasks, dist, vel_min, vel_max, inv_vel_min, inv_vel_max, idx_depot_tasks, service_time, num_tasks, idx_to_consider_r, idx_to_consider_h, idx_to_ignore_r, idx_to_ignore_h)
    global WeightHumanwaiting TimeHumFilling1 Ph num_phases dist num_agents human num_robots tasknum tasknum1 humanTime_filling num_filling_boxes num_service_tasks vel_min vel_max inv_vel_max inv_vel_min M idx_to_consider_h idx_to_ignore_h idx_to_consider_r idx_to_ignore_r first_allocation
    ReAll1 = ReAll
    num_phases = 4;
    duration = ReAll.makespan;
    valuetime = 0;
    waitingTime = [];
    first_allocation = 0;
    confirmModif = 0;
    i = 0;
    while (valuetime <= duration)

        pause(0.1); % Pause for 0.1 second
        duration = ReAll.makespan;
        i = i + 1;
        X1 = repmat(ReAll.X,num_phases,1);
        for v=1:num_robots
            orderobots{v} = [];
            index = find(X1(:,v)>0.1);
            inittime = ReAll.timeS(ReAll.X(:,v)>0.1);
            endtime = ReAll.timeF(ReAll.X(:,v)>0.1);

            length(inittime);
            for k=1:length(inittime)
                if find(index(k) == idx_going_tasks)
                    h_idx = mod(index(k), num_agents);
                    if h_idx == 0
                        h_idx = num_agents;
                    end
                    t_idx = ceil(index(k)/num_agents);
                    orderobots{v}(end+1) = h_idx;
                    orderobots{v}(end+1) = h_idx;
                end
            end
            
            curr_allocation = find(abs(X1(:,v)-1)<0.01);
            curr_timeF = ReAll.timeF(curr_allocation);
            curr_timeS = ReAll.timeS(curr_allocation);
            [~, idx_times] = sort(curr_timeF);
            agents_ordered_allocation(v).tasks = curr_allocation(idx_times);
            agents_ordered_allocation(v).timeS = curr_timeS(idx_times);
            agents_ordered_allocation(v).timeF = curr_timeF(idx_times);
            % Retrieve positions
            num_curr_tasks = length(curr_allocation);
            agents_ordered_allocation(v).type = cell(1,num_curr_tasks);
            agents_ordered_allocation(v).humanop = zeros(2,num_curr_tasks); % index human, operation human
            for k = 1:num_curr_tasks
                if find(curr_allocation(idx_times(k)) == idx_going_tasks)
                    % service task
                    h_idx = mod(curr_allocation(idx_times(k)), num_agents);
                    if h_idx == 0
                        h_idx = num_agents;
                    end
                    t_idx = ceil(curr_allocation(idx_times(k))/num_agents);
                    agents_ordered_allocation(v).type{k} = 'pick';
                    agents_ordered_allocation(v).humanop(:,k) = [h_idx, t_idx]';
                elseif find(curr_allocation(idx_times(k)) == idx_depot_tasks)
                    % depositing task
                    idxdep = curr_allocation(idx_times(k))-max(idx_going_tasks);
                    h_idx = mod(idxdep, num_agents);
                    if h_idx == 0
                        h_idx = num_agents;
                    end
                    t_idx = ceil(idxdep/num_agents);

                    agents_ordered_allocation(v).type{k} = 'depot';
                    agents_ordered_allocation(v).humanop(:,k) = [h_idx, t_idx]';
                end
            end
            
            for u=1:num_agents
                human{u}.TimeEditField.Value = valuetime;
                human{u}.TaskEditField.Value = human{u}.NumTask;
                
                %To measure the time filling
                if human{u}.StartFilling == 1
                    %BeginFill = 1;
                    human{u}.TimeStartFilling(human{u}.NumTask) = valuetime;
                    if human{u}.NumTask == 1
                        ReAll.timeSh(u) = valuetime;
                    else
                        ReAll.timeSh(u+human{u}.NumTask) = valuetime;
                        %human{u}.TimeStartFilling(human{u}.NumTask) = human{u}.TimeFinishFilling(human{u}.NumTask-1);
                        %ReAll.timeSh(u+human{u}.NumTask) = human{u}.TimeFinishFilling(human{u}.NumTask-1) + 15;
                    end
                    human{u}.StartFilling = 0;
                end
                
                if human{u}.FinishFilling == 1
                    try
                        disp('modelData1 : ')
                        modelData = rossubscriber('/timing_values', 'std_msgs/Float64')
                        %modelData = rossubscriber('/human_robot_interaction', 'diff_drive_robot/HumanRobotInteraction').LatestMessage.FinishServing
                    catch
                        try
                            disp('modelData2 : ')
                            modelData = rossubscriber('/human_robot_interaction', 'diff_drive_robot/HumanRobotInteraction').LatestMessage.FinishServing
                        catch
                            try
                                disp('modelData3 : ')
                                modelData = rossubscriber('/human_robot_interaction', 'diff_drive_robot/HumanRobotInteraction').LatestMessage.FinishServing
                            catch
                                disp('no valid message')
                                disp('modelData : ')
                            end
                        end
                    end
                    %EndFill = 1
                    human{u}.TimeFinishFilling(human{u}.NumTask) = valuetime - human{u}.TimeStartFilling(human{u}.NumTask);
                    if human{u}.NumTask == 1
                        ReAll.timeFh(u) = valuetime;
                    else
                        ReAll.timeFh(u+human{u}.NumTask) = valuetime;
                    end
                    human{u}.FinishFilling = 0;
                end

                %To measure the service time
                if human{u}.StartServing == 1
                    human{u}.TimeStartServing(human{u}.NumTask) = valuetime;
                    human{u}.StartServing = 0;
                end
                
                if human{u}.FinishServing == 1
                    human{u}.TimeFinishServing(human{u}.NumTask) = valuetime - human{u}.TimeStartServing(human{u}.NumTask);
                    human{u}.FinishServing = 0;
                end
                
                %To write the status of the robots in the human
                %apps
                if valuetime >= int64(agents_ordered_allocation(v).timeS(tasknum{v}))
                    if tasknum{v} < length(orderobots{v}) && u == orderobots{v}(tasknum{v}) 
                        if mod(tasknum{v}, 2) == 0
                            pick = 0;
                            human{u}.RobotStatusEditField.Value = string('Robot ') + num2str(v) + string(' Depositing');
                            tasknum{v} = tasknum{v} + 1;                                      
                        end
                        if mod(tasknum{v}, 1) == 0
                            pick = 1;
                            human{u}.RobotStatusEditField.Value = string('Robot ') + num2str(v) + string(' Picking');
                            tasknum{v} = tasknum{v} + 1;
                        end
                    end
                end

                if tasknum{v} > 1 && valuetime >= int64(agents_ordered_allocation(v).timeF(tasknum{v}-1)) %&& valuetime ~= int64(agents_ordered_allocation(v).timeS(tasknum{v}))
                    if pick == 1
                        human{u}.RobotStatusEditField.Value = string('Serving');
                    else
                        human{u}.RobotStatusEditField.Value = string('');
                    end          
                end

                %Update of the plan
                for k=human{u}.NumTask:num_filling_boxes
                    human{u}.TimeHumFilling(k) = human{u}.TimeFinishFilling(human{u}.NumTask);
                    human{u}.TimeFinishFilling(k) = human{u}.TimeFinishFilling(human{u}.NumTask);
                    human{u}.TimeFinishServing(k) = human{u}.TimeFinishServing(human{u}.NumTask);
                end
                TimeHumFilling1 = [];
                for i=1:num_agents
                    TimeHumFilling1(end+1) = human{i}.TimeHumFilling(i);
                end
                idx_to_consider_r = [];
                idx_to_consider_h = [];

                idx_to_ignore_r = [];
                idx_to_ignore_h = [];
                if confirmModif == 1 %if human{u}.confirmModif == 1
                    app.priority()
                    timeReall = valuetime;
                    for k=1:num_service_tasks
                        if ReAll.timeF(k) >= timeReall %tasks already started at reallocation time need to be considered
                            idx_to_consider_r(end+1) = k;
                            idx_to_consider_h(end+1) = k;
                        else
                            idx_to_ignore_r(end+1) = k;
                            idx_to_ignore_h(end+1) = k;
                        end
                    end

                    % for l=1:num_service_tasks
                    %     if ReAll.timeSh(l)> timeReall %tasks already started at reallocation time need to be considered
                    %         idx_to_consider_h(end+1) = l;
                    %     else
                    %         idx_to_ignore_h(end+1) = l;
                    %     end
                    % end
                    %% History of the humans parameters
                    waitingTime(end+1) = human{u}.alpha
                    if human{u}.NumTask >= 2
                        if waitingTime(end)>=waitingTime(end-human{u}.NumTask) && waitingTime(end)>1
                            first_allocation = 2
                            %ReAll = updateSchedule(ReAll, ReAll1, humanTime_filling, dist, vel_min, vel_max, inv_vel_min, inv_vel_max, idx_depot_tasks, idx_going_tasks, idx_to_ignore_r, idx_to_ignore_h, agents_ordered_allocation, service_time)

                            ReAll = Reallocation(num_service_tasks, num_tasks, num_agents, num_filling_boxes, num_robots, service_time, timeReall, humanTime_filling, ReAll);
                        elseif waitingTime(end)<=waitingTime(end-human{u}.NumTask) && waitingTime(end)<1
                            first_allocation = 2
                            %ReAll = updateSchedule(ReAll, ReAll1, humanTime_filling, dist, vel_min, vel_max, inv_vel_min, inv_vel_max, idx_depot_tasks, idx_going_tasks, idx_to_ignore_r, idx_to_ignore_h, agents_ordered_allocation, service_time)

                            ReAll = Reallocation(num_service_tasks, num_tasks, num_agents, num_filling_boxes, num_robots, service_time, timeReall, humanTime_filling, ReAll);
                        else
                            %ReAll = Reallocation(num_service_tasks, num_tasks, num_agents, num_filling_boxes, num_robots, service_time, timeReall, humanTime_filling, ReAll);
                            disp('update schedule one');
                            ReAll = updateSchedule(ReAll, ReAll1, humanTime_filling, dist, vel_min, vel_max, inv_vel_min, inv_vel_max, idx_depot_tasks, idx_going_tasks, idx_to_ignore_r, idx_to_ignore_h, agents_ordered_allocation, service_time)
                        end
                    end
                    if human{u}.NumTask < 2
                        first_allocation = 2
                        %ReAll = Reallocation(num_service_tasks, num_tasks, num_agents, num_filling_boxes, num_robots, service_time, timeReall, humanTime_filling, ReAll);
                        disp('update schedule two');
                        ReAll = updateSchedule(ReAll, ReAll1, humanTime_filling, dist, vel_min, vel_max, inv_vel_min, inv_vel_max, idx_depot_tasks, idx_going_tasks, idx_to_ignore_r, idx_to_ignore_h, agents_ordered_allocation, service_time)
                    end
                    human{u}.NumTask = human{u}.NumTask + 1;
                    display(ReAll, num_robots, num_agents, num_filling_boxes, idx_going_tasks, timeReall);
                    human{u}.confirmModif = 0;
                    idx_to_consider_r = [];
                    idx_to_consider_h = [];
                    idx_to_ignore_r = [];
                    idx_to_ignore_h = [];
                    confirmModif = 0;
                end
            end
            valuetime = valuetime + 1
        end
    end
end