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

    global timingData num_humans num_agents num_filling_boxes agents_ordered_allocation alreadyHere finfill finserv pub OKGOServ OKGOFill count count1

    % Set Robot ID
    MILPData{robotID}.RobotID = robotID;
    MILPData{robotID}.GoingStart = agents_ordered_allocation(robotID).timeS(1:5:length(agents_ordered_allocation(robotID).timeS));
    MILPData{robotID}.WaitingStart = agents_ordered_allocation(robotID).timeS(2:5:length(agents_ordered_allocation(robotID).timeS));
    MILPData{robotID}.ApproachingStart = agents_ordered_allocation(robotID).timeS(3:5:length(agents_ordered_allocation(robotID).timeS));
    MILPData{robotID}.ServingStart = agents_ordered_allocation(robotID).timeS(4:5:length(agents_ordered_allocation(robotID).timeS));
    MILPData{robotID}.DepotStart = agents_ordered_allocation(robotID).timeS(5:5:length(agents_ordered_allocation(robotID).timeS));

    MILPData{robotID}.GoingFinish = agents_ordered_allocation(robotID).timeF(1:5:length(agents_ordered_allocation(robotID).timeF));
    MILPData{robotID}.WaitingFinish = agents_ordered_allocation(robotID).timeF(2:5:length(agents_ordered_allocation(robotID).timeF));
    MILPData{robotID}.ApproachingFinish = agents_ordered_allocation(robotID).timeF(3:5:length(agents_ordered_allocation(robotID).timeF));
    MILPData{robotID}.ServingFinish = agents_ordered_allocation(robotID).timeF(4:5:length(agents_ordered_allocation(robotID).timeF));
    MILPData{robotID}.DepotFinish = agents_ordered_allocation(robotID).timeF(5:5:length(agents_ordered_allocation(robotID).timeF));

     
    first_row = agents_ordered_allocation(robotID).humanop(1,:);
    second_row = agents_ordered_allocation(robotID).humanop(2,:);
    MILPData{robotID}.Humans = nonzeros(first_row(second_row < 4));

    for t=1:num_filling_boxes
        MILPData{robotID}.DistanceWaiting(t) = humanData{MILPData{robotID}.Humans(t)}.RobotWaitingDistance(t);
    end

    % for hu = 1:num_humans
    %     count{hu} = 1;
    % end
    % 
    % for hu = 1:num_humans
    %     count1{hu} = 1;
    % end
    % 
    % if OKGOServ
    %     for i = 1:length(MILPData{robotID}.Humans)
    %         for j = 1:humanData{MILPData{robotID}.Humans(i)}.Task - 1
    %             if humanData{MILPData{robotID}.Humans(i)}.Robots(j) == robotID
    %                 if humanData{MILPData{robotID}.Humans(i)}.ConfirmServing(j) == 1
    %                     if count{MILPData{robotID}.Humans(i)} < humanData{MILPData{robotID}.Humans(i)}.Task
    %                         MILPData{robotID}.FinishedService(i) = 1;
    %                         count{MILPData{robotID}.Humans(i)} = count{MILPData{robotID}.Humans(i)} + 1
    %                     else
    %                         MILPData{robotID}.FinishedService(i) = 0;
    %                     end
    %                 end
    %             end
    %         end
    %     end
    % end
    % 
    % 
    % if OKGOServ
    %     for i = 1:length(MILPData{robotID}.Humans)
    %         for j = 1:humanData{MILPData{robotID}.Humans(i)}.TaskFilling - 1
    %             if humanData{MILPData{robotID}.Humans(i)}.Robots(j) == robotID
    %                 if humanData{MILPData{robotID}.Humans(i)}.ConfirmFilling(j) == 1
    %                     if count1{MILPData{robotID}.Humans(i)} < humanData{MILPData{robotID}.Humans(i)}.TaskFilling
    %                         timingData
    %                         MILPData{robotID}.FinishedFilling(i) = 1;
    %                         count1{MILPData{robotID}.Humans(i)} = count1{MILPData{robotID}.Humans(i)} + 1
    %                     else
    %                         MILPData{robotID}.FinishedFilling(i) = 0;
    %                     end
    %                 end
    %             end
    %         end
    %     end
    % end

    % for i=1:num_filling_boxes
    %     if humanData{humanID}.ConfirmFilling(i) == 1
    %         if humanData{humanID}.Robots(i) == robotID

    % if OKGOFill
    %     if humanData{MILPData{robotID}.Humans(currentHumanTask)}.Robots == robotID 
    %         if humanData{MILPData{robotID}.Humans(currentHumanTask)}.TaskFilling == 1 
    %             MILPData{robotID}.FinishedFilling(1) = 1;
    %         else
    %             MILPData{robotID}.FinishedFilling(1) = 0;
    %         end
    %     end
    % end
    %if humanData{MILPData{robotID}.Humans(MILPData{robotID}.Tasks).ConfirmFilling(humanData{MILPData{robotID}.Humans(MILPData{robotID}.Tasks)})
    if humanData{currentHumanTask}.ConfirmFilling(humanData{currentHumanTask}.TaskFilling - 1) == 1
        
   
    % Send the updated data to the corresponding ROS topic
    send(MILPDataPub{robotID}, MILPData{robotID});
    
end