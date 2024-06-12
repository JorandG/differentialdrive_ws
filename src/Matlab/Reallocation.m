function Reall = Reallocation(num_service_tasks, num_tasks, num_agents, num_filling_boxes, num_robots, service_time, timeReall, humanTime_filling, RobotID, ReAll)
    global ProxPrev inv_vel_min_prox inv_vel_max_prox Prox humanTime_serving waiting_time approaching_time humanData msgTime pubTime Ph idx_going_tasks idx_approaching_tasks idx_depot_tasks idx_waiting_tasks idx_services_tasks dist HumMaxVelRobot num_humans num_farming_robot num_agents human humanTime_filling1 WeightHumanwaiting WeightEnergyDepositing WeightEnergyProximity WeightEnergyPicking WeightMakespan vel_min vel_max inv_vel_max inv_vel_min M idx_to_consider_h idx_to_ignore_h idx_to_consider_r idx_to_ignore_r first_allocation num_phases

    serv_time = humanTime_serving;
    num_phases = 5;

    inversedistance = 0;
    distance_proximity = 1; %1m for the approaching phase distance


    idx_going_tasks = 1:num_service_tasks;
    idx_approaching_tasks = num_service_tasks+1:num_service_tasks*2;
    idx_waiting_tasks = num_service_tasks*2+1:num_service_tasks*3;
    idx_services_tasks = num_service_tasks*3+1:num_service_tasks*4;
    idx_depot_tasks = num_service_tasks*4+1:num_service_tasks*5;

    humanwaiting = 0.0;
    humanproximity = 0.0;
    velocityproximityduration = 0.0;

    %% Optimization variables

    X = optimvar('X', [num_tasks, num_robots],'Type', 'integer', 'LowerBound',0,'UpperBound',1 );
    Paux = optimvar('Paux', [(sum(1:num_tasks*num_phases)-1)*num_robots],'Type', 'integer', 'LowerBound',0,'UpperBound',1 );
    timeS = optimvar('timeS', [1 num_tasks*num_phases] ,'LowerBound', 0);
    timeF = optimvar('timeF', [1 num_tasks*num_phases] ,'LowerBound', 0);

    invprod = optimvar('invprod', [num_tasks*num_phases, num_robots] ,'LowerBound', 0);

    timeSh = optimvar('timeSh', [1 num_service_tasks] ,'LowerBound', 0);
    timeFh = optimvar('timeFh', [1 num_service_tasks] ,'LowerBound', 0);
    makespan = optimvar('makespan','LowerBound',0);%overall duration
    velocitygoing = optimvar('velocitygoing','LowerBound',0); %velocity of robot for picking
    velocityproximity = optimvar('velocityproximity', [num_tasks, num_robots] ,'LowerBound', 0); %optimvar('velocityproximity','LowerBound',0); %velocity of robot for proximity phase
    velocityproximity1 = optimvar('velocityproximity1','LowerBound',0); %velocity of robot for proximity phase
    velocitydepositing = optimvar('velocitydepositing','LowerBound',0); %velocity of robot for proximity phase
    
    max_invprod = optimvar('max_invprod', [num_tasks*num_phases], 'LowerBound', 0); %maximum of the invprod velocity

    v_min = min(vel_min);
    v_max = max(vel_max);
    normavel = (v_min*v_max)/((v_max-v_min)*num_tasks);
    normawait = 1/(num_agents*num_filling_boxes*(max(dist, [], 'all')/v_min));
    normamakespan = (makespan/(num_service_tasks*(2*max(dist, [], 'all')/v_min)+max(max(service_time(1:num_agents*num_filling_boxes,:))+max(max(service_time(1:num_agents*num_filling_boxes,:))))));
    baseProx = [];
    inv_vel_max_prox = [];
    inv_vel_min_prox = [];
    WaitWeight = [];

    for h=1:num_humans
        baseProx = [baseProx, 6 - (5 * (humanData{h}.RobotVelocityProximityWeight(humanData{RobotID}.Task) - 0.1) / 0.9)]; %coeff for the proximity phase for each human, a higher value decrease the duration of the phase and so speed up the robot
        inv_vel_max_prox = [inv_vel_max_prox, humanData{h}.RobotMaxVelocityProximity(humanData{RobotID}.Task)];
        inv_vel_min_prox = [inv_vel_min_prox, humanData{h}.RobotMinVelocityProximity(humanData{RobotID}.Task)];
        WaitWeight = [WaitWeight, humanData{h}.WaitingTimeWeight(humanData{RobotID}.Task)]; % Weight for waiting, should replace Ph
    end

    inv_vel_min_prox = repmat(inv_vel_min_prox', num_filling_boxes, 2)
    inv_vel_max_prox = repmat(inv_vel_max_prox', num_filling_boxes, 2)

    colProx = baseProx.'; 
    Prox = repmat(colProx, 1, num_robots); 
    Prox = repmat(Prox, num_filling_boxes, 1);
    %Prox = 1./Prox

    for i=1:num_agents
        humanwaiting1 = WaitWeight(i)*(sum(timeSh(num_agents+i:num_agents:num_service_tasks) - timeFh(i:num_agents:num_service_tasks-num_agents) + timeSh(i))); %app.alphaWeight(i)*
        humanwaiting = humanwaiting + humanwaiting1;
    end

    velocitydepositingduration = sum(sum(((1/min(vel_min))*X-(invprod(idx_depot_tasks,:)))))*normavel;
    velocitygoingduration = sum(sum((1/min(vel_min))*X-(invprod(idx_going_tasks,:))))*normavel;
    
    %% Problem definition
    prob = optimproblem;
    %% Objective Function
    prob.Objective =  WeightHumanwaiting*(humanwaiting)*normawait+WeightEnergyPicking*velocitygoingduration+WeightEnergyDepositing*velocitydepositingduration+WeightMakespan*normamakespan+WeightEnergyProximity*velocityproximity1;% %+WeightEnergyDepositing*(num_tasks*(1/min(vel_min))-sum(invprod,'all'));

    %% Constraints
    prob.Constraints.maxInvProd1 = max_invprod >= invprod(:,1) + invprod(:,2);
    prob.Constraints.maxInvProd12 = max_invprod <= invprod(:,1) + invprod(:,2);
    %prob.Constraints.maxInvProd3 = sum( ((1/min(vel_min))) - ((max_invprod(idx_approaching_tasks,:)).*Prox(:,1)) ) >= 0;

    prob.Constraints.makespanbound =  makespan >= timeF;


    idx_tasks = [idx_going_tasks, idx_approaching_tasks, idx_waiting_tasks, idx_services_tasks, idx_depot_tasks];
    X1 = repmat(X,num_phases,1);

    if first_allocation == 2
        
        X2 = repmat(ReAll.X,num_phases,1);
        humanTime_filling = ReAll.timeFh - ReAll.timeSh;
        idx_going_tasks_consider = idx_to_consider_r;
        idx_approaching_tasks_consider = idx_going_tasks_consider + num_service_tasks;
        idx_waiting_tasks_consider = idx_approaching_tasks_consider + num_service_tasks;
        idx_services_tasks_consider = idx_waiting_tasks_consider + num_service_tasks;
        idx_depot_tasks_consider = idx_services_tasks_consider + num_service_tasks;
        idx_to_consider_r = [idx_going_tasks_consider, idx_approaching_tasks_consider, idx_waiting_tasks_consider, idx_services_tasks_consider, idx_depot_tasks_consider];

        idx_going_tasks_ignore = idx_to_ignore_r;
        idx_approaching_tasks_ignore = idx_going_tasks_ignore + num_service_tasks;
        idx_waiting_tasks_ignore = idx_approaching_tasks_ignore + num_service_tasks;
        idx_services_tasks_ignore = idx_waiting_tasks_ignore + num_service_tasks;
        idx_depot_tasks_ignore = idx_services_tasks_ignore + num_service_tasks;
        idx_to_ignore_r = [idx_going_tasks_ignore, idx_approaching_tasks_ignore, idx_waiting_tasks_ignore, idx_services_tasks_ignore, idx_depot_tasks_ignore];
        prob.Constraints.velocityproximityduration11 = velocityproximity(idx_going_tasks_consider,:) == ((inv_vel_min_prox(idx_going_tasks_consider,:)).*X2(idx_going_tasks_consider,:)) - ((invprod(idx_approaching_tasks_consider,:)).*Prox(idx_going_tasks_consider,:)); %sum(sum( ((invprod(idx_approaching_tasks,:).*Prox - (1/min(vel_min))*X)) ))
        prob.Constraints.velocityproximityduration12 = velocityproximity1 == sum( sum( velocityproximity) )*normavel;

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
        
        prob.Constraints.duration7 = invprod(idx_approaching_tasks_consider,:) >= X1(idx_approaching_tasks_consider,:).*inv_vel_max_prox(idx_going_tasks_consider,:);
        prob.Constraints.duration8 = invprod(idx_approaching_tasks_consider,:) <= X1(idx_approaching_tasks_consider,:).*inv_vel_min_prox(idx_going_tasks_consider,:);

        prob.Constraints.duration11 = invprod(idx_to_ignore_r,:) == ReAll.invprod(idx_to_ignore_r,:);
        prob.Constraints.duration12 = max_invprod(idx_to_ignore_r,:) == ReAll.max_invprod(idx_to_ignore_r,:);
        prob.Constraints.maxInvProd3 = sum( (inv_vel_min_prox(idx_going_tasks_consider,1)) - ((max_invprod(idx_approaching_tasks_consider,:)).*Prox(idx_going_tasks_consider,1)) ) >= 0;

        prob.Constraints.timeF_tasks_to_ignore = timeF(idx_to_ignore_r) == ReAll.timeF(idx_to_ignore_r);
        prob.Constraints.timeS_tasks_to_ignore = timeS(idx_to_ignore_r) == ReAll.timeS(idx_to_ignore_r);
        prob.Constraints.X_tasks_to_ignore = X1(idx_to_ignore_r,:) == X2(idx_to_ignore_r,:);

        prob.Constraints.approaching_time = timeS(idx_approaching_tasks_consider) == timeF(idx_going_tasks_consider); %approaching human
        prob.Constraints.approaching_time1 = timeF(idx_approaching_tasks_consider) == timeF(idx_going_tasks_consider) + max_invprod(idx_approaching_tasks_consider)'*distance_proximity; %invprod(idx_approaching_tasks_consider,:)/l_approaching;%(timeSh(num_humans+1:length(idx_waiting_tasks_consider))-timeF(idx_services_tasks_consider(1:length(idx_waiting_tasks_consider)-num_humans)))/(max(dist(:))/min(vel_min) + serv_time)

        prob.Constraints.waiting_time = timeS(idx_waiting_tasks_consider) == timeF(idx_approaching_tasks_consider); %waiting for human
        prob.Constraints.waiting_time1 = timeF(idx_waiting_tasks_consider) == timeS(idx_waiting_tasks_consider) + 0.0;%(timeF(idx_approaching_tasks_consider)-timeS(idx_services_tasks_consider))%waiting_time;%(timeSh(num_humans+1:length(idx_waiting_tasks_consider))-timeF(idx_services_tasks_consider(1:length(idx_waiting_tasks_consider)-num_humans)))/(max(dist(:))/min(vel_min) + 7);

        prob.Constraints.services_time = timeS(idx_services_tasks_consider) == timeF(idx_waiting_tasks_consider); %time to make the service to the human
        prob.Constraints.services_time2 = timeF(idx_services_tasks_consider) == timeS(idx_services_tasks_consider) + serv_time(idx_going_tasks_consider); %time to make the service to the human using idx going because we can't exceed num_taks
        prob.Constraints.depot_time = timeS(idx_depot_tasks_consider) == timeF(idx_services_tasks_consider);

        prob.Constraints.humanduration1 = timeFh(idx_to_ignore_h) == ReAll.timeFh(idx_to_ignore_h);
        prob.Constraints.humanduration3 = timeSh(idx_to_ignore_h) == ReAll.timeSh(idx_to_ignore_h);
        prob.Constraints.humanduration5 = timeFh(idx_to_consider_h) == timeSh(idx_to_consider_h) + humanTime_filling(idx_to_consider_h);

    elseif first_allocation == 0
        prob.Constraints.velocityproximityduration11 = velocityproximity == ((1/min(vel_min))*X) - ((invprod(idx_approaching_tasks,:)).*Prox); %sum(sum( ((invprod(idx_approaching_tasks,:).*Prox - (1/min(vel_min))*X)) ))
        prob.Constraints.velocityproximityduration12 = velocityproximity1 == sum( sum( velocityproximity) )*normavel;
        
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

        prob.Constraints.duration7 = invprod(idx_approaching_tasks,:) >= X1(idx_approaching_tasks,:).*repmat((inv_vel_max)', num_tasks,1);
        prob.Constraints.duration8 = invprod(idx_approaching_tasks,:) <= X1(idx_approaching_tasks,:).*repmat((inv_vel_min)', num_tasks,1);

        prob.Constraints.approaching_time01 = timeF(idx_approaching_tasks(1:num_humans)) == timeF(idx_going_tasks(1:num_humans)) + max_invprod(idx_approaching_tasks(1:num_humans))'*distance_proximity;
        prob.Constraints.approaching_time02 = timeS(idx_approaching_tasks(1:num_humans)) == timeF(idx_going_tasks(1:num_humans));
        prob.Constraints.approaching_time = timeS(idx_approaching_tasks(num_humans+1:length(idx_approaching_tasks))) == timeF(idx_going_tasks(num_humans+1:length(idx_approaching_tasks))); 
        prob.Constraints.approaching_time1 = timeF(idx_approaching_tasks(num_humans+1:length(idx_approaching_tasks))) == timeS(idx_approaching_tasks(num_humans+1:length(idx_approaching_tasks))) + max_invprod(idx_approaching_tasks(num_humans+1:length(idx_approaching_tasks)),:)'*distance_proximity;%(timeSh(num_humans+1:length(idx_waiting_tasks))-timeF(idx_services_tasks(1:length(idx_waiting_tasks)-num_humans)))/(max(dist(:))/min(vel_min) + serv_time)

        prob.Constraints.waiting_time = timeS(idx_waiting_tasks) == timeF(idx_approaching_tasks); 
        prob.Constraints.waiting_time2 = timeF(idx_waiting_tasks) == timeS(idx_waiting_tasks) + (timeF(idx_approaching_tasks)-timeS(idx_services_tasks(1:length(idx_waiting_tasks))));%-timeFh(1:length(idx_waiting_tasks)))*((min(vel_min)./dist(idx_going_tasks)) + serv_time(idx_going_tasks))';

        prob.Constraints.services_time = timeS(idx_services_tasks) == timeF(idx_waiting_tasks); %time to make the service to the human
        prob.Constraints.services_time2 = timeF(idx_services_tasks) == timeS(idx_services_tasks) + serv_time; %time to make the service to the human
        prob.Constraints.depot_time = timeS(idx_depot_tasks) == timeF(idx_services_tasks);

        prob.Constraints.humanduration = timeFh == timeSh + humanTime_filling;

        prob.Constraints.maxInvProd3 = sum( ((1/min(vel_min))) - ((max_invprod(idx_approaching_tasks,:)).*Prox(:,1)) ) >= 0;
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
                parallelism(r) = timeS(j)-timeF(i)>=-M*(2-X(i,k)-X1(j,k))-M*(1-Paux(l)); r=r+1;
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