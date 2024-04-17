addpath('/opt/gurobi1000/linux64/matlab');
savepath;

addpath('~/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/install/m')
savepath

clear all;
close all

global humanTime_fillingPrev timingData TimingSub MILPData MILPDataPub sizeFinishFilling msgTime pubTime pub msg humanSub humanData Ph human num_farming_robot tasknum tasknum1 num_robots initialTime num_filling_boxes idx_going_tasks idx_depot_tasks idx_waiting_tasks idx_services_tasks dist  num_humans VelRob HumWait HumTimeList num_service_tasks num_tasks num_depot_tasks service_time num_agents humanTime_filling humanTime_filling1 WeightHumanwaiting WeightEnergyPicking WeightMakespan vel_min vel_max inv_vel_max inv_vel_min M idx_to_consider_h idx_to_ignore_h idx_to_consider_r idx_to_ignore_r first_allocation

num_humans = 2;
num_agents = num_humans;
num_filling_boxes = 3;
num_robots = 2;
initialTime = 150;
humanData = cell(num_humans, 1);
pub = cell(num_humans, 1);
humanSub = cell(num_humans, 1);
sizeFinishFilling = 1;
home = 1;
sim = 0;

if ros.internal.Global.isNodeActive == 0
    if home == 1
        setenv('ROS_HOSTNAME','localhost')
        rosinit('http://localhost:11311');
    else
        setenv('ROS_HOSTNAME','192.168.0.137')
        rosinit('http://192.168.0.137:11311');
    end
end

if sim == 0
    rosparam set /use_sim_time false;
end

for r=1:num_robots
    MILPDataPub{r} = rospublisher(sprintf("/MILPResults%d", r), 'diff_drive_robot/MILPResult');
    MILPData{r} = rosmessage(MILPDataPub{r});
    MILPData{r}.RobotID = 0;
    MILPData{r}.Humans = [];
    MILPData{r}.GoingStart = [];
    MILPData{r}.ApproachingStart = [];
    MILPData{r}.WaitingStart = [];
    MILPData{r}.ServingStart = [];
    MILPData{r}.DepotStart = [];
    MILPData{r}.GoingFinish = [];
    MILPData{r}.ApproachingFinish = [];
    MILPData{r}.WaitingFinish = [];
    MILPData{r}.ServingFinish = [];
    MILPData{r}.DepotFinish = [];
    MILPData{r}.FinishedFilling = [0, 0, 0];
    MILPData{r}.FinishedService = [0, 0, 0];
    %send(MILPDataPub{r}, MILPData{r});
end

for h=1:num_humans
    humanData{h} = rossubscriber(sprintf("/human_robot_interaction%d", h), 'diff_drive_robot/HumanRobotInteraction').LatestMessage;
    humanSub{h} = rossubscriber(sprintf("/human_robot_interaction%d", h), 'diff_drive_robot/HumanRobotInteraction');
    pub{h} = rospublisher(sprintf("/human_robot_interaction%d", h), 'diff_drive_robot/HumanRobotInteraction');
    humanData{h} = rosmessage(pub{h});
    humanData{h}.HumanID = h;
    humanData{h}.RobotVelocity = 0.0;
    humanData{h}.WaitingTime = 0.0;
    humanData{h}.StartFilling = [0.0, 167.0, 334.0];%, 486.0];
    humanData{h}.FinishFilling = [150.0, 317.0, 484.0];%, 636];
    humanData{h}.TimeFilling = ones(1, num_filling_boxes)*150;
    humanData{h}.TimeServing = ones(1, num_filling_boxes)*7.0;
    humanData{h}.StartServing = 0.0;
    humanData{h}.FinishServing = 0.0;
    humanData{h}.Task = 1;
    send(pub{h}, humanData{h});
end 

for i=1:num_agents
    for j=1:num_filling_boxes
        human{i}.TimeFinishFilling = [];
        human{i}.TimeStartFilling = [];
        human{i}.TimeStartServing = [];
        human{i}.TimeFinishServing = [];
        human{i}.TimeHumFilling =  [];  
    end
end

for i=1:num_agents
    Ph(i) = 1;
    human{i}.HumanID = humanData{i}.HumanID;
    human{i}.alpha = humanData{i}.WaitingTime;
    for j=1:num_filling_boxes
        human{i}.TimeFinishFilling(end+1) = initialTime;
        human{i}.TimeStartFilling(end+1) = 1;
        human{i}.TimeStartServing(end+1) = 0;
        human{i}.TimeFinishServing(end+1) = 15;
        human{i}.TimeHumFilling(end+1) = initialTime;
        human{i}.NumTask = 1; 
        humanData{i}.StartFilling;
        humanData{i}.FinishFilling;
        humanData{i}.StartServing;
        humanData{i}.FinishServing;
    end
end

for j=1:num_robots
    tasknum{j} = 1;
    tasknum1{j} = 1;
end

num_service_tasks = num_filling_boxes*num_agents;
num_depot_tasks = num_service_tasks;
num_tasks = num_service_tasks;
service_time = zeros(num_tasks, num_robots);

M = 1000000;
vel_min = ones(num_robots,1)*0.05; % min velocity for the robots
vel_max = ones(num_robots,1)*0.2; % max velocity for the robots
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
probot = [-1, -0.5; 0, 0.5];
position_hum = [2, 3; 1, 2];

ptasks = repmat(position_hum.', 1, num_filling_boxes);
[dist, service_time] = computeDist(num_tasks, num_robots, position_hum, ptasks, num_service_tasks, probot, idx_going_tasks, service_time);
timeReall = 0;
idx_to_consider_h = [];
idx_to_consider_r = [];
idx_to_ignore_h = [];
idx_to_ignore_r = [];
first_allocation = 0;
ReAll = 0;
humanTime_filling = ones(1, num_agents*num_filling_boxes)*150; %Find the first paramerters randomly for the allocation
humanTime_fillingPrev = humanTime_filling;

drawnow
ReAll = Reallocation(num_service_tasks, num_tasks, num_agents, num_filling_boxes, num_robots, service_time, timeReall, humanTime_filling, ReAll);
display(ReAll, num_robots, num_agents, num_filling_boxes, idx_going_tasks, timeReall);

% Command to run ROS node for timing
command = 'source /opt/ros/noetic/setup.bash && rosnode list | grep "^/timer_publisher" | xargs -I {} rosnode kill {}';
[status, cmdout] = system(command); % Shutting down old timer
disp('Starting new timer node: ');
command = 'source /home/jorand/differentialdrive_ws/devel/setup.bash && rosrun diff_drive_robot timing.py &';
[status, cmdout] = system(command, '-echo'); % Starting new timer
TimingSub = rossubscriber('/Timing', 'std_msgs/Float64');
timingData = receive(TimingSub, 1).Data
for hu=1:num_humans
    HumanID = hu;
    hum1(HumanID); %human app
end
simulation(ReAll, idx_going_tasks, dist, vel_min, vel_max, inv_vel_min, inv_vel_max, idx_depot_tasks, service_time, num_tasks, idx_to_consider_r, idx_to_consider_h, idx_to_ignore_r, idx_to_ignore_h)

function Reall = Reallocation(num_service_tasks, num_tasks, num_agents, num_filling_boxes, num_robots, service_time, timeReall, humanTime_filling, ReAll)
    global humanData msgTime pubTime Ph idx_going_tasks idx_approaching_tasks idx_depot_tasks idx_waiting_tasks idx_services_tasks dist HumMaxVelRobot num_humans num_farming_robot num_agents human humanTime_filling1 WeightHumanwaiting WeightEnergyPicking WeightMakespan vel_min vel_max inv_vel_max inv_vel_min M idx_to_consider_h idx_to_ignore_h idx_to_consider_r idx_to_ignore_r first_allocation num_phases

    serv_time = 7;
    approaching_time = 5;
    waiting_time = 5;
    num_phases = 5;

    inversedistance = 0;
    inversedistancegoing = 0;
    inversedistancedepot= 0;

    idx_going_tasks = 1:num_service_tasks;
    idx_approaching_tasks = num_service_tasks+1:num_service_tasks*2;
    idx_waiting_tasks = num_service_tasks*2+1:num_service_tasks*3;
    idx_services_tasks = num_service_tasks*3+1:num_service_tasks*4;
    idx_depot_tasks = num_service_tasks*4+1:num_service_tasks*5;

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

    % for i=1:num_agents
    %     humanwaiting1 = Ph(i)*(sum(timeSh(num_agents+i:num_agents:num_service_tasks) - timeFh(i:num_agents:num_service_tasks-num_agents) + timeSh(i))); %app.alphaWeight(i)*
    %     humanwaiting = humanwaiting + humanwaiting1;
    % end

    %% Problem definition
    prob = optimproblem;
    prob.Objective =  WeightHumanwaiting*(humanwaiting)+WeightEnergyPicking*velocitypicking+WeightMakespan*normamakespan; %+WeightEnergyDepositing*(num_tasks*(1/min(vel_min))-sum(invprod,'all'));

    %% Constraints
    prob.Constraints.makespanbound =  makespan >= timeF;
    prob.Constraints.velocitypickingduration1 = velocitypicking == (num_tasks*(1/min(vel_min))-sum(invprod,'all'))*normavel;

    idx_tasks = [idx_going_tasks, idx_approaching_tasks, idx_waiting_tasks, idx_services_tasks, idx_depot_tasks];
    X1 = repmat(X,num_phases,1);

    if first_allocation == 2
        X2 = repmat(ReAll.X,num_phases,1);
        %humanTime_filling = ReAll.timeFh - ReAll.timeSh
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
        prob.Constraints.humanduration5 = timeFh(idx_to_consider_h) == timeSh(idx_to_consider_h) + humanTime_filling(idx_to_consider_h);

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

        prob.Constraints.approaching_time01 = timeF(idx_approaching_tasks(1:num_humans)) == timeF(idx_going_tasks(1:num_humans)) + approaching_time;
        prob.Constraints.approaching_time02 = timeS(idx_approaching_tasks(1:num_humans)) == timeF(idx_going_tasks(1:num_humans));
        prob.Constraints.approaching_time = timeS(idx_approaching_tasks(num_humans+1:length(idx_approaching_tasks))) == timeF(idx_going_tasks(num_humans+1:length(idx_approaching_tasks))); 
        prob.Constraints.approaching_time1 = timeF(idx_approaching_tasks(num_humans+1:length(idx_approaching_tasks))) == timeS(idx_approaching_tasks(num_humans+1:length(idx_approaching_tasks))) + approaching_time;%(timeSh(num_humans+1:length(idx_waiting_tasks))-timeF(idx_services_tasks(1:length(idx_waiting_tasks)-num_humans)))/(max(dist(:))/min(vel_min) + serv_time)

        prob.Constraints.waiting_time = timeS(idx_waiting_tasks) == timeF(idx_approaching_tasks); 
        prob.Constraints.waiting_time2 = timeF(idx_waiting_tasks) == timeS(idx_waiting_tasks) + waiting_time; 

        prob.Constraints.services_time = timeS(idx_services_tasks) == timeF(idx_waiting_tasks); %time to make the service to the human
        prob.Constraints.services_time2 = timeF(idx_services_tasks) == timeS(idx_services_tasks) + serv_time; %time to make the service to the human
        prob.Constraints.depot_time = timeS(idx_depot_tasks) == timeF(idx_services_tasks);

        prob.Constraints.humanduration = timeFh == timeSh + humanTime_filling;
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

function AllUpdated = updateSchedule(All, Alloc, humanTime_filling, dist, vel_min, vel_max, inv_vel_min, inv_vel_max, idx_depot_tasks, idx_going_tasks, idx_to_ignore_r, idx_to_ignore_h, agents_ordered_allocation, service_time, humanTime_fillingPrev)
    global difference humanTime_fillingPrev timeReall num_tasks num_robots num_agents human num_filling_boxes num_phases 

    service_time1 = [];
    serv_time = 7;
    waiting_time = 0;
    X = repmat(All.X,num_phases,1); 
    for u=1:num_agents
        count = u;
        for v=1:num_filling_boxes
            service_time1(count) = human{u}.TimeFinishServing(v);
            count = count + num_agents;
        end
    end
    All1 = All;
    dist1 = dist;
    dist1 = repmat(dist1,num_phases,1);

    %Shifting human timings (to do before updating robot times)
    for curr_hum=1:num_tasks
        %Shifts human timings
        if All.timeFh(curr_hum)==timeReall
            for s=curr_hum+num_agents:num_agents:num_tasks 
                All.timeSh(s) = All.timeSh(s) + difference + 10;
                difference = abs(All.timeFh(s) - (All.timeSh(s) + humanTime_filling(curr_hum)));  
                All.timeFh(s) = All.timeSh(s) + humanTime_filling(curr_hum);
            end
        end
    end

    for i=1:num_tasks
        curr_hum = i;
        checkShift = false;
        curr_hum_slow = false;
        if (ismember(curr_hum,idx_going_tasks))
            dep = curr_hum+num_tasks*(num_phases-1);
            serv = curr_hum+num_tasks*(num_phases-2);
            wait = curr_hum+num_tasks*(num_phases-3);
        end

        if All.timeFh(curr_hum)==timeReall
            %human slower
            if humanTime_filling(curr_hum) > humanTime_fillingPrev(curr_hum)
                disp('slower')
                curr_hum_slow = true;
                hum_slow = curr_hum;
                checkShift = true;
            end
        end

        prev_dep_time = All.timeF(dep);

        %if All.timeF(curr_hum) < All.timeFh(curr_hum) + service_time(curr_hum) || dist1(curr_hum) ~= dist(curr_hum) && ~ismember(curr_hum, idx_to_ignore_h) %If the robot wants to serve the human before he has finished
        if ~ismember(curr_hum, idx_to_ignore_h)
            prev_timeF_curr_hum = All.timeF(curr_hum);
            if dist1(curr_hum)/(All.timeS(curr_hum)-All.timeFh(curr_hum))<max(vel_max) && curr_hum_slow %If speed up the going possible
                    disp('speed up going possible')
                    All.timeS(hum_slow) = All.timeFh(hum_slow) - nonzeros(dist1(hum_slow,:).*All.X(hum_slow,:))/max(vel_max); %We speed up to the max velocity to minimize the shifting quantity
                    All.timeF(hum_slow) = All.timeFh(hum_slow); %+ service_time1(hum_slow); %update final time of going
                    All.timeS(wait)=All.timeF(hum_slow); %waiting
                    All.timeF(wait)=All.timeS(wait)+waiting_time;
                    All.timeS(serv)=All.timeF(wait); %serving
                    All.timeF(serv)=All.timeS(serv)+serv_time;%service_time1(hum_slow);
                    All.timeS(dep)=All.timeF(serv); %depot
                    All.timeF(dep)=All.timeS(dep)+nonzeros(dist1(dep,:).*X(dep,:))/max(vel_max);%update final time of depot
                %end            else %Speed up not possible
                disp('speed up NOT possible')
                All.timeS(curr_hum) = All.timeFh(curr_hum) - (nonzeros(All.invprod(curr_hum,:))*nonzeros(dist1(curr_hum,:).*All.X(curr_hum,:)))
                All.timeF(curr_hum) = All.timeFh(curr_hum); %+ service_time1(curr_hum); %update final time of picking

                All.timeS(wait)=All.timeF(curr_hum); %waiting
                All.timeF(wait)=All.timeS(wait)+waiting_time;
                All.timeS(serv)=All.timeF(wait); %serving
                All.timeF(serv)=All.timeS(serv)+serv_time;%service_time(dep);
                All.timeS(dep)=All.timeF(serv); %depot
                All.timeF(dep)=All.timeS(dep)+(nonzeros(All.invprod(dep,:))*nonzeros(dist1(dep,:).*X(dep,:)));%update final time of depot
            end

            variation = (All.timeF(dep) - prev_dep_time) 

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
        elseif ismember(curr_hum, idx_to_ignore_h) && curr_hum_slow
            disp('shifting current human')
            deltar = All.timeFh(curr_hum) - All.timeF(curr_hum);
            All.timeF(curr_hum) = All.timeFh(curr_hum); %+ service_time1(hum_slow); %update final time of going
            All.timeS(wait) = All.timeS(wait) + deltar; %waiting
            All.timeF(wait) = All.timeF(wait) + deltar;
            All.timeS(serv) = All.timeS(serv) + deltar; %serving
            All.timeF(serv) = All.timeF(serv) + deltar;%service_time1(hum_slow);
            All.timeS(dep) = All.timeS(dep) + deltar; %depot
            All.timeF(dep) = All.timeF(dep) + deltar;%update final time of depot
        end

    end

    All.makespan = max(All.timeF);
    AllUpdated = All;
end

function display(ReAll, num_robots, num_agents, num_filling_boxes, idx_going_tasks, timeReAll)
    global num_phases
    %% Display
    %close all
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
    global humanData TimeHumFilling1 Ph num_phases dist num_agents human num_robots tasknum tasknum1 humanTime_filling num_filling_boxes num_service_tasks vel_min vel_max inv_vel_max inv_vel_min M idx_to_consider_h idx_to_ignore_h idx_to_consider_r idx_to_ignore_r first_allocation
    TimeHumFilling1
    for i=1:num_agents
        Ph(i) = human{i}.TimeHumFilling(i)/150 + humanData{i}.RobotVelocity;
    end
end

function sendRobotTaskUpdates(robotID, ReAll, X1, MILPDataPub, MILPData, idx_going_tasks, num_filling_boxes, humanData, currentHumanTask)
    % sendRobotTaskUpdates - Update and send task information for a specific robot
    %
    % Syntax:  sendRobotTaskUpdates(robotID, ReAll, X1, MILPDataPub, MILPData, idx_going_tasks, num_filling_boxes, humanData, currentHumanTask)
    %
    % Inputs:
    %    robotID - ID of the robot
    %    ReAll - Struct containing all reallocation data
    %    X1 - Task assignment matrix for all robots
    %    MILPDataPub - Array of ROS publishers for each robot
    %    MILPData - Array of ROS messages for each robot
    %    idx_going_tasks - Index array for 'going' tasks
    %    num_filling_boxes - Number of filling boxes (used for indexing task phases)
    %    humanData - Cell array of human data messages
    %    currentHumanTask - Current task index for the human
    %
    % Outputs:
    %    none

    global num_agents;

    % Identify tasks assigned to this robot
    index = find(X1(:, robotID) > 0.1);
    inittime = ReAll.timeS(X1(:, robotID) > 0.1);
    endtime = ReAll.timeF(X1(:, robotID) > 0.1);

    % Initialize human indexing for this robot's tasks
    z = 1;
    for k = 1:length(inittime)
        if any(index(k) == idx_going_tasks)
            MILPData{robotID}.Humans(z) = mod(index(k), num_agents);
            if MILPData{robotID}.Humans(z) == 0
                MILPData{robotID}.Humans(z) = num_agents;
            end
            z = z + 1;
        end
    end

    % Set Robot ID
    MILPData{robotID}.RobotID = robotID;
    
    % Assign start and finish times for task phases
    MILPData{robotID}.GoingStart = inittime(1:num_filling_boxes);
    MILPData{robotID}.ApproachingStart = inittime(num_filling_boxes+1:2*num_filling_boxes);
    MILPData{robotID}.WaitingStart = inittime(2*num_filling_boxes+1:3*num_filling_boxes);
    MILPData{robotID}.ServingStart = inittime(3*num_filling_boxes+1:4*num_filling_boxes);
    MILPData{robotID}.DepotStart = inittime(4*num_filling_boxes+1:5*num_filling_boxes);

    MILPData{robotID}.GoingFinish = endtime(1:num_filling_boxes);
    MILPData{robotID}.ApproachingFinish = endtime(num_filling_boxes+1:2*num_filling_boxes);
    MILPData{robotID}.WaitingFinish = endtime(2*num_filling_boxes+1:3*num_filling_boxes);
    MILPData{robotID}.ServingFinish = endtime(3*num_filling_boxes+1:4*num_filling_boxes);
    MILPData{robotID}.DepotFinish = endtime(4*num_filling_boxes+1:5*num_filling_boxes);
    MILPData{robotID}.FinishedFilling(currentHumanTask) = 1;

    % Send the updated data to the corresponding ROS topic
    send(MILPDataPub{robotID}, MILPData{robotID});
end


function simulation(ReAll, idx_going_tasks, dist, vel_min, vel_max, inv_vel_min, inv_vel_max, idx_depot_tasks, service_time, num_tasks, idx_to_consider_r, idx_to_consider_h, idx_to_ignore_r, idx_to_ignore_h)
    global  idx_to_consider_current_r difference humanTime_fillingPrev timeReall num_humans timingData TimingSub MILPData MILPDataPub sizeFinishFilling pub msg humanSub humanData msgTime pubTime WeightHumanwaiting TimeHumFilling1 Ph num_phases dist num_agents human num_robots tasknum tasknum1 humanTime_filling num_filling_boxes num_service_tasks vel_min vel_max inv_vel_max inv_vel_min M idx_to_consider_h idx_to_ignore_h idx_to_consider_r idx_to_ignore_r first_allocation
    ReAll1 = ReAll
    num_phases = 5;
    duration = ReAll.makespan;
    valuetime = 0;
    waitingTime = [];
    first_allocation = 0;
    confirmModif = 0;
    newDatas = 0;
    i = 0;
    FirstSendMILPResults = 1;

    while (valuetime <= duration)
        h=0;
        for h=1:num_agents
            humanData{h} = humanSub{h}.LatestMessage;
            while true
                pause(0.1); % Avoid overloading with too frequent requests
                timingData = receive(TimingSub, 1).Data;
                try
                    humanData{h} = humanSub{h}.LatestMessage;
                    if ~isempty(humanData{h}) % Example condition
                        break; 
                    else
                        disp('Invalid or incomplete data received. Waiting for next message...');
                    end
                catch ME
                    disp('Error while waiting for humanData. Trying again...');
                end
            end
        end
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
                idx_to_consider_r = [];
                idx_to_consider_h = [];

                idx_to_ignore_r = [];
                idx_to_ignore_h = [];

                if humanData{u}.Confirm == 1 
                    %% ROS topic for the robots
                    sendRobotTaskUpdates(v, ReAll, X1, MILPDataPub, MILPData, idx_going_tasks, num_filling_boxes, humanData, humanData{u}.Task)
                    % %% ROS topic for the robots
                    % index = find(X1(:,v)>0.1);
                    % inittime = ReAll.timeS(X1(:,v)>0.1);
                    % endtime = ReAll.timeF(X1(:,v)>0.1);
                    % z = 1;
                    % for k=1:length(inittime)
                    %     if find(index(k) == idx_going_tasks)
                    %         MILPData{v}.Humans(z) = mod(index(k), num_agents);
                    %         if MILPData{v}.Humans(z) == 0
                    %             MILPData{v}.Humans(z) = num_agents;
                    %         end
                    %     end
                    %     z = z + 1;
                    % end
                    % MILPData{v}.RobotID = v;
                    % 
                    % %Start timings
                    % MILPData{v}.GoingStart = inittime(1:num_filling_boxes);
                    % MILPData{v}.ApproachingStart = inittime(num_filling_boxes+1:num_filling_boxes*2);
                    % MILPData{v}.WaitingStart = inittime(num_filling_boxes*2+1:num_filling_boxes*3);
                    % MILPData{v}.ServingStart = inittime(num_filling_boxes*3+1:num_filling_boxes*4);
                    % MILPData{v}.DepotStart = inittime(num_filling_boxes*4+1:num_filling_boxes*5);
                    % 
                    % %Finish timings
                    % MILPData{v}.GoingFinish = endtime(1:num_filling_boxes);
                    % MILPData{v}.ApproachingFinish = endtime(num_filling_boxes+1:num_filling_boxes*2);
                    % MILPData{v}.WaitingFinish = endtime(num_filling_boxes*2+1:num_filling_boxes*3);
                    % MILPData{v}.ServingFinish = endtime(num_filling_boxes*3+1:num_filling_boxes*4);
                    % MILPData{v}.DepotFinish = endtime(num_filling_boxes*4+1:num_filling_boxes*5);
                    % MILPData{v}.FinishedFilling(humanData{u}.Task) = 1;
                    % 
                    % send(MILPDataPub{v}, MILPData{v});
                    % 
                    %% ROS topics for the humans
                    humanData{u}.Confirm = 0;
                    newDatas = 1;
                    timeReall = timingData;
                    difference = abs(timeReall - humanData{u}.FinishFilling(humanData{u}.Task)); %used for the updateschedule function
                    humanData{u}.FinishFilling(humanData{u}.Task) = timeReall;

                    for h=1:num_agents
                        humanTime_filling(h:num_humans:end) = humanData{h}.FinishFilling - humanData{h}.StartFilling;
                        ReAll.timeFh(h:num_humans:end) = humanData{h}.FinishFilling; 
                        ReAll.timeSh(h:num_humans:end) = humanData{h}.StartFilling;
                    end    
                    send(pub{u}, humanData{u});

                    for h=1:num_agents
                        humanTime_filling(h:num_humans:end) = humanData{h}.FinishFilling - humanData{h}.StartFilling;
                        ReAll.timeFh(h:num_humans:end) = humanData{h}.FinishFilling; 
                    end

                    l=0;
                    for l=1:num_service_tasks
                        if ReAll.timeFh(l) > timeReall 
                            idx_to_consider_h(end+1) = l;
                            idx_to_consider_r(end+1) = l;
                        else
                            idx_to_ignore_h(end+1) = l;
                            idx_to_ignore_r(end+1) = l;
                        end
                    end

                    %% History of the humans parameters
                    if humanData{u}.Task >= 2
                        disp('update schedule two');
                        first_allocation = 2
                        idx_to_consider_r
                        % Need to run updateSchedule before Reallocation
                        ReAll = updateSchedule(ReAll, ReAll1, humanTime_filling, dist, vel_min, vel_max, inv_vel_min, inv_vel_max, idx_depot_tasks, idx_going_tasks, idx_to_ignore_r, idx_to_ignore_h, agents_ordered_allocation, service_time, humanTime_fillingPrev);
                        %ReAll = Reallocation(num_service_tasks, num_tasks, num_agents, num_filling_boxes, num_robots, service_time, timeReall, humanTime_filling, ReAll);                        
                        humanTime_fillingPrev = humanTime_filling;
                        % Send new timings on the topic
                        humanData{u}.FinishFilling = ReAll.timeFh(u:2:end);
                        humanData{u}.StartFilling = ReAll.timeSh(u:2:end);
                        if humanData{u}.Task > 2
                            humanData{u}.Task = 3;
                        else
                            humanData{u}.Task = humanData{u}.Task + 1;
                        end
                        send(pub{u}, humanData{u});
                    end

                    if humanData{u}.Task < 2
                        disp('update schedule one');
                        first_allocation = 2
                        % Need to run updateSchedule before Reallocation
                        ReAll = updateSchedule(ReAll, ReAll1, humanTime_filling, dist, vel_min, vel_max, inv_vel_min, inv_vel_max, idx_depot_tasks, idx_going_tasks, idx_to_ignore_r, idx_to_ignore_h, agents_ordered_allocation, service_time, humanTime_fillingPrev);
                        %ReAll = Reallocation(num_service_tasks, num_tasks, num_agents, num_filling_boxes, num_robots, service_time, timeReall, humanTime_filling, ReAll);                        
                        humanTime_fillingPrev = humanTime_filling;
                        % Send new timings on the topic
                        humanData{u}.FinishFilling = ReAll.timeFh(u:2:end);
                        humanData{u}.StartFilling = ReAll.timeSh(u:2:end);
                        humanData{u}.Task = humanData{u}.Task + 1;
                        send(pub{u}, humanData{u});    
                    end

                    display(ReAll, num_robots, num_agents, num_filling_boxes, idx_going_tasks, timeReall);
                    human{u}.confirmModif = 0;
                    idx_to_consider_r = [];
                    idx_to_consider_h = [];
                    idx_to_ignore_r = [];
                    idx_to_ignore_h = [];
                    confirmModif = 0;
                end
            end
        end
        valuetime = timingData;
        disp(['valuetime : ', num2str(valuetime)]);            
    end
end

