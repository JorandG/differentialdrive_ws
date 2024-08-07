addpath('/opt/gurobi1000/linux64/matlab');
savepath;

addpath('~/differentialdrive_ws/src/matlab_msg_gen_ros1/glnxa64/install/m')
savepath

clear all;
close all;

global Reduction chi ProxPrev inv_vel_min_prox inv_vel_max_prox ProximityTaskDurations ProximityTaskVelocities ProximityTaskFeedback ProximityTaskWeights Prox ReAllSave approaching_time waiting_time humanTime_serving humanTime_fillingPrev timingData TimingSub MILPData MILPDataPub sizeFinishFilling msgTime pubTime pub msg humanSub humanData Ph human num_farming_robot tasknum tasknum1 num_robots initialTime num_filling_boxes idx_going_tasks idx_depot_tasks idx_waiting_tasks idx_approaching_tasks idx_services_tasks dist  num_humans VelRob HumWait HumTimeList num_service_tasks num_tasks num_depot_tasks service_time num_agents humanTime_filling humanTime_filling1 WeightHumanwaiting WeightEnergyProximity WeightEnergyDepositing WeightEnergyPicking WeightMakespan vel_min vel_max inv_vel_max inv_vel_min M idx_to_consider_h idx_to_ignore_h idx_to_consider_r idx_to_ignore_r first_allocation

num_humans = 2;
num_agents = num_humans;
num_filling_boxes = 3;
num_robots = 2;
initialTime = 75;
humanData = cell(num_humans, 1);
pub = cell(num_humans, 1);
humanSub = cell(num_humans, 1);
sizeFinishFilling = 1;
%waiting_time = 5;
approaching_time = 5;
serving_time = 7;
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
    MILPData{r}.FinishedFilling = repmat(0, 1, num_filling_boxes+1);
    MILPData{r}.FinishedService = repmat(0, 1, num_filling_boxes+1);
    %send(MILPDataPub{r}, MILPData{r});
end

for h=1:num_humans
    humanData{h} = rossubscriber(sprintf("/human_robot_interaction%d", h), 'diff_drive_robot/HumanRobotInteraction').LatestMessage;
    humanSub{h} = rossubscriber(sprintf("/human_robot_interaction%d", h), 'diff_drive_robot/HumanRobotInteraction');
    pub{h} = rospublisher(sprintf("/human_robot_interaction%d", h), 'diff_drive_robot/HumanRobotInteraction');
    humanData{h} = rosmessage(pub{h});
    humanData{h}.HumanID = h;

    humanData{h}.Robots = repmat(1, 1, num_filling_boxes);
    humanData{h}.RobotWaitingDistance = repmat(2, 1, num_filling_boxes); %Initialized with 2 meters
    humanData{h}.RobotVelocityProximity = repmat(1, 1, num_filling_boxes); %Initialized with 0.1m/s
    humanData{h}.RobotMinVelocityProximity = repmat(20, 1, num_filling_boxes); %inv_vel_min 1/0.0875: 75% of the physical v_min
    humanData{h}.RobotMaxVelocityProximity = repmat(3.33333, 1, num_filling_boxes); %inv_vel_max 1/0.15: 75% of the physical v_max
    humanData{h}.RobotVelocityProximityWeight = repmat(0.2, 1, num_filling_boxes);
    humanData{h}.WaitingTime = repmat(1, 1, num_filling_boxes); %Initialized with 1, it's the weight mapping to the Objective Function directly
    humanData{h}.WaitingTimeWeight = repmat(0.2, 1, num_filling_boxes);

    for i = 1:num_filling_boxes, humanData{h}.StartFilling(i) = sum([0, repmat([initialTime + approaching_time + serving_time], 1, i-1)]); end
    for i = 1:num_filling_boxes, humanData{h}.FinishFilling(i) = sum([initialTime, repmat([initialTime + approaching_time + serving_time], 1, i-1)]); end
    humanData{h}.TimeFilling = ones(1, num_filling_boxes)*initialTime*h;
    humanData{h}.TimeServing = repmat(serving_time, 1, num_filling_boxes);
    humanData{h}.StartServing = humanData{h}.FinishFilling + approaching_time;
    humanData{h}.FinishServing = humanData{h}.StartServing + serving_time;
    humanData{h}.ConfirmServing = [0, 0, 0, 0];%repmat(0, 1, num_filling_boxes);
    humanData{h}.ConfirmFilling = [0, 0, 0, 0];%repmat(0, 1, num_filling_boxes);
    humanData{h}.Task = 1;
    humanData{h}.TaskFilling = 1;
    humanData{h}.Happiness = [0, 0, 0, 0];
    send(pub{h}, humanData{h});
end 

for j=1:num_robots
    tasknum{j} = 1;
    tasknum1{j} = 1;
end

num_service_tasks = num_filling_boxes*num_agents;
num_depot_tasks = num_service_tasks;
num_tasks = num_service_tasks;
service_time = zeros(num_tasks, num_robots);

M = 2000000;
vel_min = ones(num_robots,1)*0.05; % min velocity for the robots
vel_max = [0.2; 0.3];%ones(num_robots,1)*0.2; % max velocity for the robots
chi = ones(num_robots,1)*1;
Reduction = 1;

% Weights for objective function
WeightHumanwaiting = 1;
WeightEnergyPicking = 0.1;
WeightEnergyProximity = 0.1;
WeightEnergyDepositing = 0.1;
WeightMakespan = 0.1;
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
RobotID = 1;
%humanTime_filling = ones(1, num_agents*num_filling_boxes)*150; %Find the first paramerters randomly for the allocation
humanTime_filling = [75, 150, 75, 150, 75, 150]
humanTime_serving = ones(1, num_agents*num_filling_boxes)*7.0;
humanTime_fillingPrev = humanTime_filling;

% Define weight combinations based on DOE
alpha_vals = [0.1, 1.0];
zeta_vals = [0.1, 1.0];
kappa_vals = [0.1, 1.0];
num_experiments = length(alpha_vals) *2* length(zeta_vals) *2* length(kappa_vals);

results = zeros(num_experiments, 10); % Matrix to store results for each experiment

experiment_num = 1;

for alpha_h1 = alpha_vals
    for zeta_h1 = zeta_vals
        for alpha_h2 = alpha_vals
            for zeta_h2 = zeta_vals
                for kappa = kappa_vals
                    % Set weights for current experiment
                    for h = 1:num_humans
                        if h == 1
                            humanData{h}.WaitingTimeWeight(:) = alpha_h1;
                            humanData{h}.RobotVelocityProximityWeight(:) = zeta_h1;
                        else
                            humanData{h}.WaitingTimeWeight(:) = alpha_h2;
                            humanData{h}.RobotVelocityProximityWeight(:) = zeta_h2;
                        end
                        send(pub{h}, humanData{h});
                    end
                    WeightMakespan = kappa;
                    % Run the Reallocation function
                    ReAll = 0;
                    timeReAll = 0;
                    ReAll = Reallocation(num_service_tasks, num_tasks, num_agents, num_filling_boxes, num_robots, service_time, timeReall, humanTime_filling, RobotID, ReAll);
                    % Extract response variables
                    for h = 1:num_agents
                        % Find the indices of robots serving the human
                        serving_robots = find(ReAll.X(:, h));
                        % Calculate the actual robot numbers (considering num_robots)
                        robot_numbers = mod(serving_robots - 1, num_robots) + 1;
                        
                        % Store the results in the corresponding human vectors
                        humanData{h}.Robots = robot_numbers';
                    end

                    % Display the results
                    for h = 1:num_agents
                        fprintf('Human %d robots:\n', h);
                        disp(humanData{h}.Robots);
                        send(pub{h}, humanData{h});
                    end

                    w_a1_o = sum(ReAll.timeSh(num_agents+1:num_agents:num_service_tasks) - ReAll.timeFh(1:num_agents:num_service_tasks-num_agents));
                    w_a2_o = sum(ReAll.timeSh(num_agents+2:num_agents:num_service_tasks) - ReAll.timeFh(2:num_agents:num_service_tasks-num_agents));
                    v_p_a1 = mean(1./ReAll.max_invprod(idx_approaching_tasks(1:num_humans:num_tasks)));
                    v_p_a2 = mean(1./ReAll.max_invprod(idx_approaching_tasks(2:num_humans:num_tasks)));
                    robots_h1 = humanData{1}.Robots;
                    robots_h2 = humanData{2}.Robots;
                    delta = ReAll.makespan;

                    % Store results
                    results(experiment_num, :) = [alpha_h1, zeta_h1, alpha_h2, zeta_h2, kappa, w_a1_o, v_p_a1,  w_a2_o, v_p_a2, delta]
                    experiment_num = experiment_num + 1
                end
            end
        end
    end
end

% Display the results
disp('Results:');
disp('alpha_h1  zeta_h1  alpha_h2  zeta_h2  kappa  w_a1_o  v_p_a1  v_p_a2  w_a2_o  delta');
disp(results);

% Save results to a file
save('experiment_results.mat', 'results');
