function sendRobotTaskUpdates(robotID, humanID, ReAll, X1, MILPDataPub, MILPData, idx_going_tasks, num_filling_boxes, humanData, currentHumanTask)

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

    global humanData num_agents

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

    if humanData{humanID}.ConfirmServing == 1 
        if MILPData{robotID}.Humans(currentHumanTask) == humanID
            MILPData{robotID}.FinishedService(currentHumanTask) = 1;  
        end
    end
    if humanData{humanID}.ConfirmFilling == 1 
        if MILPData{robotID}.Humans(currentHumanTask) == humanID
            MILPData{robotID}.FinishedFilling(currentHumanTask) = 1;
        end
    end

    % Send the updated data to the corresponding ROS topic
    send(MILPDataPub{robotID}, MILPData{robotID});
    
end