addpath('/opt/gurobi1000/linux64/matlab');
savepath;

addpath('~/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/install/m')
savepath

clear all;
close all;

global approaching_time waiting_time humanTime_serving humanTime_fillingPrev timingData TimingSub MILPData MILPDataPub sizeFinishFilling msgTime pubTime pub msg humanSub humanData Ph human num_farming_robot tasknum tasknum1 num_robots initialTime num_filling_boxes idx_going_tasks idx_depot_tasks idx_waiting_tasks idx_approaching_tasks idx_services_tasks dist  num_humans VelRob HumWait HumTimeList num_service_tasks num_tasks num_depot_tasks service_time num_agents humanTime_filling humanTime_filling1 WeightHumanwaiting WeightEnergyProximity WeightEnergyDepositing WeightEnergyPicking WeightMakespan vel_min vel_max inv_vel_max inv_vel_min M idx_to_consider_h idx_to_ignore_h idx_to_consider_r idx_to_ignore_r first_allocation

num_humans = 2;
num_agents = num_humans;
num_filling_boxes = 3;
num_robots = 2;
initialTime = 150;
humanData = cell(num_humans, 1);
pub = cell(num_humans, 1);
humanSub = cell(num_humans, 1);
sizeFinishFilling = 1;
waiting_time = 5.0;
approaching_time = 5.0;
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
    humanData{h}.TimeServing = [7.0, 7.0, 7.0, 7.0];
    humanData{h}.StartServing = humanData{h}.FinishFilling + waiting_time + approaching_time;
    humanData{h}.FinishServing = humanData{h}.StartServing + 7.0;
    humanData{h}.ConfirmServing == 0;
    humanData{h}.ConfirmFilling == 0;
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

M = 100000;
vel_min = ones(num_robots,1)*0.05; % min velocity for the robots
vel_max = ones(num_robots,1)*0.2; % max velocity for the robots
% Weights for objective function
WeightHumanwaiting = 1;
WeightEnergyPicking = 1;
WeightEnergyProximity = 2;
WeightEnergyDepositing = 1;
WeightMakespan = 10;
std_dev = 140;
inv_vel_min = 1./vel_min;
inv_vel_max = 1./vel_max;

idx_going_tasks = 1:num_service_tasks;
idx_approaching_tasks = num_service_tasks+1:num_service_tasks*2;
idx_waiting_tasks = num_service_tasks*2+1:num_service_tasks*3;
idx_services_tasks = num_service_tasks*3+1:num_service_tasks*4;
idx_depot_tasks = num_service_tasks*4+1:num_service_tasks*5;

% Positions
probot = [-1, -0.5; 0, 0.5];
position_hum = [12, 3; 10, 2; 11, 5; 8, 7];%[2, 3; 1, 2];

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
humanTime_serving = ones(1, num_agents*num_filling_boxes)*7.0;
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

function simulation(ReAll, idx_going_tasks, dist, vel_min, vel_max, inv_vel_min, inv_vel_max, idx_depot_tasks, service_time, num_tasks, idx_to_consider_r, idx_to_consider_h, idx_to_ignore_r, idx_to_ignore_h)
    global humanTime_serving FirstSendMILPResults idx_to_consider_current_r difference humanTime_fillingPrev timeReall num_humans timingData TimingSub MILPData MILPDataPub sizeFinishFilling pub msg humanSub humanData msgTime pubTime WeightHumanwaiting TimeHumFilling1 Ph num_phases dist num_agents human num_robots tasknum tasknum1 humanTime_filling num_filling_boxes num_service_tasks vel_min vel_max inv_vel_max inv_vel_min M idx_to_consider_h idx_to_ignore_h idx_to_consider_r idx_to_ignore_r first_allocation
    ReAll1 = ReAll
    num_phases = 5;
    duration = ReAll.makespan;
    valuetime = 0;
    waitingTime = [];
    first_allocation = 0;
    confirmModif = 0;
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

                %% Sends ROS topic for the robots for the initialization
                if FirstSendMILPResults == 1
                    sendRobotTaskUpdates(v, u, ReAll, X1, MILPDataPub, MILPData, idx_going_tasks, num_filling_boxes, humanData, humanData{u}.Task)
                end
                    
                if humanData{u}.ConfirmFilling == 1
                    sendRobotTaskUpdates(v, u, ReAll, X1, MILPDataPub, MILPData, idx_going_tasks, num_filling_boxes, humanData, humanData{u}.Task)
                    humanData{u}.ConfirmFilling = 0
                    send(pub{u}, humanData{u});
                end

                if humanData{u}.ConfirmServing == 1 
                   
                    %% ROS topic for the robots
                    sendRobotTaskUpdates(v, u, ReAll, X1, MILPDataPub, MILPData, idx_going_tasks, num_filling_boxes, humanData, humanData{u}.Task) 
                    humanData{u}.ConfirmServing = 0
                    send(pub{u}, humanData{u});
                    timeReall = timingData;
                    difference = (humanData{u}.FinishFilling(humanData{u}.Task) - humanData{u}.StartFilling(humanData{u}.Task)) - human{u}.TimeHumFilling(humanData{u}.Task) %abs(timeReall - humanData{u}.FinishFilling(humanData{u}.Task)); %used for the updateschedule function
                    %humanData{u}.FinishFilling(humanData{u}.Task) = timeReall;

                    for h=1:num_agents
                        humanTime_filling(h:num_humans:end) = humanData{h}.FinishFilling - humanData{h}.StartFilling;
                        humanTime_serving(h:num_humans:end) = humanData{h}.FinishServing - humanData{h}.StartServing;
                        ReAll.timeFh(h:num_humans:end) = humanData{h}.FinishFilling; 
                        ReAll.timeSh(h:num_humans:end) = humanData{h}.StartFilling;
                    end    
                    send(pub{u}, humanData{u});

                    % for h=1:num_agents
                    %     humanTime_filling(h:num_humans:end) = humanData{h}.FinishFilling - humanData{h}.StartFilling;
                    %     ReAll.timeFh(h:num_humans:end) = humanData{h}.FinishFilling; 
                    % end

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
                        for c=1:num_robots
                            sendRobotTaskUpdates(c, u, ReAll, X1, MILPDataPub, MILPData, idx_going_tasks, num_filling_boxes, humanData, humanData{u}.Task)
                        end
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
                        for c=1:num_robots
                            sendRobotTaskUpdates(c, u, ReAll, X1, MILPDataPub, MILPData, idx_going_tasks, num_filling_boxes, humanData, humanData{u}.Task)
                        end                    
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
        FirstSendMILPResults = 0;
        valuetime = timingData;
        disp(['valuetime : ', num2str(valuetime)]);            
    end
end

