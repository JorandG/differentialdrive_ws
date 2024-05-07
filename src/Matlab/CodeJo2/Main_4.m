% ---------------------------------------------------------------------------- %
%                                     Reset                                    %
% ---------------------------------------------------------------------------- %
rosshutdown;
close all;
clc;
clear;
clear global;
% ---------------------------------------------------------------------------- %

% ----------------------------- Global Parameters ---------------------------- %
global plan;
global gazeboModelStates;
global num_robots; num_robots = 2;
global spline_interpolation; spline_interpolation = 1;
global robot;

simulation = 1;

warehouse_position = [-0.9, 0.1; -1, 0.8];

b = 0.15;
distanceThreshold = 0.06; % 6 cm

obstacleAvoidanceEnabled = false;
obstacleAvoidanceThreshold = 0.5; % 1 m
obstacleAvoidanceGain = 1.5;
obstacleAvoidanceEta = 1.01;
% ---------------------------------------------------------------------------- %

% ---------------------------------------------------------------------------- %
%                                  Connection                                  %
% ---------------------------------------------------------------------------- %
if ros.internal.Global.isNodeActive == 0
    if simulation == 1
        setenv('ROS_HOSTNAME','localhost')
        rosinit('http://localhost:11311');
    else
        setenv('ROS_HOSTNAME','192.168.0.137')
        rosinit('http://192.168.0.137:11311');
    end
end
% ---------------------------------------------------------------------------- %

% ---------------------------------------------------------------------------- %
%                                  Declaration                                 %
% ---------------------------------------------------------------------------- %

% ------------------------------------ ROS ----------------------------------- %
publishers_and_subscribers_definition(simulation);
% ---------------------------------------------------------------------------- %

% ----------------------------------- Robot ---------------------------------- %
robots_parameters_and_controllers_initialization(simulation,b,distanceThreshold,warehouse_position);
% ---------------------------------------------------------------------------- %

% ----------------------------------- Plan ----------------------------------- %
plan_processing();
% ---------------------------------------------------------------------------- %

% ----------------------------------- Path ----------------------------------- %
for i=1:num_robots
    % ---------------------------------- Timers ---------------------------------- %
    robot{i}.start_time.local.updated = 0;
    robot{i}.start_time.local.value = 0;

    robot{i}.start_time.global.initialized = 0;
    robot{i}.start_time.global.value = 0;
    % ---------------------------------------------------------------------------- %
end
% ---------------------------------------------------------------------------- %

% ----------------------------------- Time ----------------------------------- %
frequency =  50;
Ts = 1/frequency;

loop_rate = rateControl(frequency);
r = rosrate(frequency);

time.start_time = -1;
time.elapsed_time = 0.0;
% ---------------------------------------------------------------------------- %

% ----------------------------------- Goal ----------------------------------- %
% for i=1:num_robots
%     robot{i}.goal.initial_orientation = robot{i}.pose(1,3);
%     robot{i}.goal.initial_position =  [robot{i}.controller.y_1,robot{i}.controller.y_2];

%     robot{i}.goal.corner_orientation =  robot{i}.waypoints.orientations(1,:);
%     robot{i}.goal.corner_position =  robot{i}.waypoints.positions(1,:);
%     robot{i}.goal.corner_position(1,1) = robot{i}.goal.corner_position(1,1) + (robot{i}.controller.b * cos(robot{i}.goal.corner_orientation));
%     robot{i}.goal.corner_position(1,2) = robot{i}.goal.corner_position(1,2) + (robot{i}.controller.b * sin(robot{i}.goal.corner_orientation));
    
%     robot{i}.goal.final_orientation = robot{i}.waypoints.orientations(2,:);
%     robot{i}.goal.final_position = robot{i}.waypoints.positions(2,:);
%     robot{i}.goal.final_position(1,1) = robot{i}.goal.final_position(1,1) + (robot{i}.controller.b * cos(robot{i}.goal.final_orientation));
%     robot{i}.goal.final_position(1,2) = robot{i}.goal.final_position(1,2) + (robot{i}.controller.b * sin(robot{i}.goal.final_orientation));
    

%     robot{i}.goal.displacement = robot{i}.goal.final_position - robot{i}.goal.initial_position;
%     robot{i}.goal.instants = [0.0,robot{i}.waypoints.times'];

%     robot{i}.goal.final_waypoint_reached = 0;
% end
% ---------------------------------------------------------------------------- %

% % ---------------------------------- Figure ---------------------------------- %
% figure
% title('Planned L-shaped Trajectory');
% grid on;
% hold on;
% xlabel('X (m)');
% ylabel('Y (m)');
% 
% % ----------------------------- Find Limit Values ---------------------------- %
% x_values = [warehouse_position(:,1)',desired_positions(:,1)'];
% y_values = [warehouse_position(:,2)',desired_positions(:,2)'];
% 
% for i=1:num_robots
%     x_values(1,end + 1) = robot{i}.pose(:,1);
%     y_values(1,end + 1) = robot{i}.pose(:,2);
% end
% % ---------------------------------------------------------------------------- %
% 
% for i=1:num_robots
%     plot(robot{i}.waypoints.positions(:,1), robot{i}.waypoints.positions(:,2), '-o', 'LineWidth', 2);
%     xlim([min(x_values,[],'all') - 1, max(x_values,[],'all') + 1]);
%     ylim([min(y_values,[],'all') - 1, max(y_values,[],'all') + 1]);
% end
% % ---------------------------------------------------------------------------- %
% 
% % ---------------------------------------------------------------------------- %


% ---------------------------------- Figure ---------------------------------- %
% figure
% title('Planned L-shaped Trajectory');
% grid on;
% hold on;
% xlabel('X (m)');
% ylabel('Y (m)');

% x_values = desired_positions(:,1)';
% y_values = desired_positions(:,2)';

% for i=1:num_robots
%     x_values(1,end + 1) = robot{i}.home_position(1,1);
%     y_values(1,end + 1) = robot{i}.home_position(1,2);
% end

% xlim([min(x_values,[],'all') - 1, max(x_values,[],'all') + 1]);
% ylim([min(y_values,[],'all') - 1, max(y_values,[],'all') + 1]);

% for i=1:num_robots
%     plot([robot{i}.home_position(1,1), robot{i}.waypoints.positions(:,1)'], [robot{i}.home_position(1,2), robot{i}.waypoints.positions(:,2)'], '-o', 'LineWidth', 2);
%     plot(robot{i}.q.actual(:,1), robot{i}.q.actual(:,2), '-*', 'Color', "#77AC30");
% end
% ---------------------------------------------------------------------------- %

% ---------------------------------------------------------------------------- %
%                            Object for Rate Control                           %
% ---------------------------------------------------------------------------- %

% ---------------------------------------------------------------------------- %


% ---------------------------------------------------------------------------- %
%                                   Main Loop                                  %
% ---------------------------------------------------------------------------- %
while ros.internal.Global.isNodeActive
    % ---------------------------------------------------------------------------- %
    %                                  Time Check                                  %
    % ---------------------------------------------------------------------------- %
    if time.start_time < 0
        % ----------------------------------- Reset ---------------------------------- %
        reset(r);
        % ---------------------------------------------------------------------------- %

        % ----------------------------------- Time ----------------------------------- %
        time.start_time = r.TotalElapsedTime;
        % ---------------------------------------------------------------------------- %
    end
    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %
    %                                  i-th Robot                                  %
    % ---------------------------------------------------------------------------- %
    for i=1:num_robots
        % ---------------------------------------------------------------------------- %
        %                              Auxiliary Variables                             %
        % ---------------------------------------------------------------------------- %
        if (isempty(robot{i}.ongoing_activity.activity) == 0)
            curren_phase_ID = robot{i}.ongoing_activity.ongoing_phase;
            allocated_time = robot{i}.ongoing_activity.activity.phase{curren_phase_ID}.allocated_time;
            goal = robot{i}.ongoing_activity.activity.phase{curren_phase_ID}.goal;
        end
        % ---------------------------------------------------------------------------- %

        % ---------------------------------------------------------------------------- %
        %                                     Pose                                     %
        % ---------------------------------------------------------------------------- %
        robot{i}.odometry.Data = robot{i}.odometry.Subscriber.LatestMessage;
        robot{i}.pose = odometry_robot_pose_sim(robot{i}.odometry.Data.Pose);

        robot{i}.controller.y_1 = robot{i}.pose(1,1) + (robot{i}.controller.b * cos(robot{i}.pose(1,3)));
        robot{i}.controller.y_2 = robot{i}.pose(1,2) + (robot{i}.controller.b * sin(robot{i}.pose(1,3)));
        % ---------------------------------------------------------------------------- %

        % ---------------------------------------------------------------------------- %
        %                          Ongoing Activity - Updating                         %
        % ---------------------------------------------------------------------------- %
        if (isempty(robot{i}.ongoing_activity.activity) == 0)
            % ---------------------------------------------------------------------------- %
            %                                Position Error                                %
            % ---------------------------------------------------------------------------- %
            robot{i}.controller.error = goal.final_position - [robot{i}.controller.y_1, robot{i}.controller.y_2];
            % ---------------------------------------------------------------------------- %

            % ---------------------------------------------------------------------------- %
            %                                     Goal                                     %
            % ---------------------------------------------------------------------------- %
            if (((goal.type) == 0 || (goal.type) == 1) && (norm(robot{i}.controller.error) <= robot{i}.controller.distance_threshold) && (time.elapsed_time >= goal.instants(1,3)) || (goal.type) == -1 && (time.elapsed_time >= goal.instants(1,2)))
                % ----------------------------------- State ---------------------------------- %
                robot{i}.ongoing_activity.activity.phase{curren_phase_ID}.goal.completed = 1;
                robot{i}.ongoing_activity.activity.phase{curren_phase_ID}.goal.plots.end_index = length(time.elapsed_time);
                % ---------------------------------------------------------------------------- %

                % ------------------------ Are there any other goals? ------------------------ %
                if (length(robot{i}.ongoing_activity.activity.phase) > curren_phase_ID)
                    % -------------------- Updating of the Current Phase's ID -------------------- %
                    curren_phase_ID = curren_phase_ID + 1;
                    robot{i}.ongoing_activity.ongoing_phase = curren_phase_ID;
                    % ---------------------------------------------------------------------------- %

                    % ----------------------------- Goal Construction ---------------------------- %            
                    goal = build_goal(robot{i}.ongoing_activity.activity.phase{curren_phase_ID}.type, ...
                                      spline_interpolation,robot{i}.controller.b, ...
                                      robot{i}.pose,0.0,robot{i}.ongoing_activity.activity.phase{curren_phase_ID}.desired_configuration, ...
                                      robot{i}.ongoing_activity.activity.phase{curren_phase_ID}.allocated_time,time.elapsed_time);
                                      
                    goal.plots.start_index = length(time.elapsed_time);
                    % ---------------------------------------------------------------------------- %

                    % -------------------------------- Allocation -------------------------------- %
                    robot{i}.ongoing_activity.activity.phase{curren_phase_ID}.goal = goal;
                    % ---------------------------------------------------------------------------- %

                    % ---------------------------------- Message --------------------------------- %
                    disp(['[Robot-', num2str(i), ']: A New Goal has been allocated!']);
                    % ---------------------------------------------------------------------------- %
                else
                    % -------------------------- Ongoing Activity State -------------------------- %s
                    robot{i}.ongoing_activity.completed = 1;
                    % ---------------------------------------------------------------------------- %

                    % --------------------------------- Dismissal -------------------------------- %
                    robot{i}.completed_activities{end + 1} = robot{i}.ongoing_activity.activity;
                    % ---------------------------------------------------------------------------- %

                    % ----------------------------------- Reset ---------------------------------- %
                    robot{i}.ongoing_activity.activity = {}; goal = {}; allocated_time = 0.0;
                    robot{i}.ongoing_activity.ongoing_phase = 1; curren_phase_ID = robot{i}.ongoing_activity.ongoing_phase;
                    robot{i}.ongoing_activity.completed = 0;
                    % ---------------------------------------------------------------------------- %
                end
                % ---------------------------------------------------------------------------- %
            end
            % ---------------------------------------------------------------------------- %
        elseif (length(robot{i}.activities) > 0) && (robot{i}.activities{1}.start_time >= time.elapsed_time)
            % ----------------------------- Goal Construction ---------------------------- %            
            goal = build_goal(robot{i}.activities{1}.phase{1}.type,spline_interpolation,robot{i}.controller.b, ...
                              robot{i}.pose,pi,robot{i}.activities{1}.phase{1}.desired_configuration, ...
                              robot{i}.activities{1}.phase{1}.allocated_time,time.elapsed_time);
                              
            goal.plots.start_index = length(time.elapsed_time);
            % ---------------------------------------------------------------------------- %

            % -------------------------------- Allocation -------------------------------- %
            robot{i}.ongoing_activity.activity = robot{i}.activities{1};
            robot{i}.ongoing_activity.activity.phase{1}.goal = goal;
            % ---------------------------------------------------------------------------- %

            % ------------------- Updating of the Activities Structure ------------------- %
            if (length(robot{i}.activities) - 1) > 0
                robot{i}.activities = robot{i}.activities{2:end};
            else
                robot{i}.activities = {};
            end
            % ---------------------------------------------------------------------------- %

            % -------------------------- Reset of the Parameters ------------------------- %
            robot{i}.ongoing_activity.ongoing_phase = 1; curren_phase_ID = 1; allocated_time = robot{i}.ongoing_activity.activity.phase{1}.allocated_time;
            robot{i}.ongoing_activity.completed = 0;
            % ---------------------------------------------------------------------------- %

            % ---------------------------------- Message --------------------------------- %
            disp(['[Robot-', num2str(i), ']: An Activity has been allocated!']);
            % ---------------------------------------------------------------------------- %
        else
            % ---------------------------------- Message --------------------------------- %
            disp(['[Robot-', num2str(i), ']: There are no activities ongoing! (', num2str(robot{i}.controller.error), ')']);
            % ---------------------------------------------------------------------------- %
        end
        % ---------------------------------------------------------------------------- %

        % ---------------------------------------------------------------------------- %
        %                         Ongoing Activity - Processing                        %
        % ---------------------------------------------------------------------------- %
        if ((isempty(robot{i}.ongoing_activity.activity) == 0) && ((goal.type) == 0 || (goal.type) == 1))
            % ---------------------- Trajectory Generation (Spline) ---------------------- %
            [traj,dtraj] = spline_trajectory(goal.initial_position(1,1:2), ... 
                                             goal.corner_position(1,1:2), ...
                                             goal.final_position(1,1:2), ...
                                             goal.instants(1,1), ...
                                             goal.instants(1,2), ...
                                             goal.instants(1,3), ...
                                             time.elapsed_time);
            % ---------------------------------------------------------------------------- %

            % ------------------------ Input/Output Linearization ------------------------ %
            robot{i}.controller.y_1d = traj(1,1);
            robot{i}.controller.y_2d = traj(1,2);
            
            robot{i}.controller.dy_1d = dtraj(1,1);
            robot{i}.controller.dy_2d = dtraj(1,2);

            robot{i}.controller.dy_1 = robot{i}.controller.dy_1d + robot{i}.controller.k1*(robot{i}.controller.y_1d - robot{i}.controller.y_1);
            robot{i}.controller.dy_2 = robot{i}.controller.dy_2d + robot{i}.controller.k2*(robot{i}.controller.y_2d - robot{i}.controller.y_2);
            % ---------------------------------------------------------------------------- %

            % -------------------------------- Velocities -------------------------------- %
            velocities = [cos(robot{i}.pose(1,3)) sin(robot{i}.pose(1,3)); -(sin(robot{i}.pose(1,3))/robot{i}.controller.b) (cos(robot{i}.pose(1,3))/robot{i}.controller.b)]*[robot{i}.controller.dy_1 robot{i}.controller.dy_2]';

            robot{i}.linearVelocity = velocities(1,1);
            robot{i}.angularVelocity = velocities(2,1);

            robot{i}.linearVelocity = sign(robot{i}.linearVelocity)*min(abs(robot{i}.linearVelocity),0.25);
            robot{i}.angularVelocity = sign(robot{i}.angularVelocity)*min(abs(robot{i}.angularVelocity),1.0);
            % ---------------------------------------------------------------------------- %
        else
            % -------------------------------- Velocities -------------------------------- %
            robot{i}.controller.dy_1 = 0.0; robot{i}.controller.dy_2 = 0.0;
            robot{i}.controller.dy_1d = 0.0; robot{i}.controller.dy_2d = 0.0;

            robot{i}.linearVelocity = 0.0; robot{i}.angularVelocity = 0.0;
            % ---------------------------------------------------------------------------- %
        end
        % ---------------------------------------------------------------------------- %

        % ---------------------------------------------------------------------------- %
        %                                     State                                    %
        % ---------------------------------------------------------------------------- %
        robot{i}.q.nominal(end+1,:) = [robot{i}.controller.y_1d robot{i}.controller.y_2d];
        robot{i}.q.actual(end+1,:) = [robot{i}.controller.y_1 robot{i}.controller.y_2];

        robot{i}.dq.nominal(end+1,:) = [robot{i}.controller.dy_1d robot{i}.controller.dy_2d];
        robot{i}.dq.actual(end+1,:) = [robot{i}.controller.dy_1, robot{i}.controller.dy_2];

        robot{i}.time(end+1,1) = time.elapsed_time;
        % ---------------------------------------------------------------------------- %
    end
    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %
    %                                 ROS Messages                                 %
    % ---------------------------------------------------------------------------- %
    for i=1:num_robots
        vel_msg = rosmessage(robot{i}.velocities_publisher);
        vel_msg.Linear.X = robot{i}.linearVelocity;
        vel_msg.Angular.Z = robot{i}.angularVelocity;
        send(robot{i}.velocities_publisher, vel_msg);
    end
    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %
    %                                     Sleep                                    %
    % ---------------------------------------------------------------------------- %
    waitfor(loop_rate);
    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %
    %                                 Elapsed Time                                 %
    % ---------------------------------------------------------------------------- %
    time.elapsed_time = r.TotalElapsedTime - time.start_time;
    time.elapsed_time
    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %
    %                                 ROS SHUTDOWN                                 %
    % ---------------------------------------------------------------------------- %
    for i=1:num_robots
        if ((isempty(robot{i}.ongoing_activity.activity) == 0) ||(length(robot{i}.activities) > 0))
            break;
        elseif i == num_robots
            rosshutdown;
            % ---------------------------------------------------------------------------- %
            %                                     Plots                                    %
            % ---------------------------------------------------------------------------- %
            for i=1:num_robots
                % ---------------------------- State and Velocity ---------------------------- %
                robot_plots(robot{i}.time,robot{i}.q.nominal,robot{i}.q.actual,robot{i}.dq.nominal,robot{i}.dq.actual,i);
                % ---------------------------------------------------------------------------- %

                % --------------------------------- Waypoints -------------------------------- %
                waypoints{i} = [robot{i}.completed_activities{1}.phase{1}.goal.initial_position; ...
                                robot{i}.completed_activities{1}.phase{1}.goal.corner_position; ...
                                robot{i}.completed_activities{1}.phase{1}.goal.final_position];
                % ---------------------------------------------------------------------------- %

                % ----------------------------------- State ---------------------------------- %
                q{i} = robot{i}.q.actual;
                q_des{i} = robot{i}.q.nominal;
                % ---------------------------------------------------------------------------- %

                % ----------------------------------- Plot ----------------------------------- %
                if i == num_robots
                    trajectory_plot(waypoints,q,[]);
                    trajectory_plot(waypoints,[],q_des);
                end
                % ---------------------------------------------------------------------------- %
            end
            % ---------------------------------------------------------------------------- %
        end
    end
    % ---------------------------------------------------------------------------- %

    % for i=1:num_robots
    %     % ---------------------------------------------------------------------------- %
    %     %                              Auxiliary Variables                             %
    %     % ---------------------------------------------------------------------------- %
    %     curren_phase_ID = robot{i}.ongoing_activity.ongoing_phase;
    %     allocated_time = robot{i}.ongoing_activity.activity.phase{curren_phase_ID}.allocated_time;
    %     goal = robot{i}.ongoing_activity.activity.phase{curren_phase_ID}.goal;
    %     % ---------------------------------------------------------------------------- %



    %     % ---------------------------------------------------------------------------- %
    %     %                              Goal/Activity State                             %
    %     % ---------------------------------------------------------------------------- %

    %     % -------------------------------- Goal State -------------------------------- %
    %     if ((goal.type) == 0 && (norm(robot{i}.controller.error) <= robot{i}.controller.distance_threshold)) ||((goal.type) == 1 && (goal.instants(1,3) - goal.instants(1,1)) >= allocated_time)
    %         robot{i}.ongoing_activity.activity.phase{curren_phase_ID}.goal.completed = 1;
    %         goal = robot{i}.ongoing_activity.activity.phase{curren_phase_ID}.goal;
    %     end
    %     % ---------------------------------------------------------------------------- %

    %     % --------------------------------- Next Goal -------------------------------- %
    %     if (goal.completed == 1)
    %         if (curren_phase_ID < length(robot{i}.ongoing_activity.activity.phase))
    %             % ------------------------------- Ongoing Phase ------------------------------ %
    %             curren_phase_ID = curren_phase_ID + 1;
    %             robot{i}.ongoing_activity.ongoing_phase = curren_phase_ID;
    %             % ---------------------------------------------------------------------------- %

    %             % ----------------------------- Goal Construction ---------------------------- %            
    %             goal = build_goal(robot{i}.activities{1}.phase{1}.type,spline_interpolation,robot{i}.controller.b, ...
    %                               robot{i}.pose,0.0,robot{i}.activities{1}.phase{1}.desired_configuration, ...
    %                               robot{i}.activities{1}.phase{1}.allocated_time,time.elapsed_time);                  
    %             % ---------------------------------------------------------------------------- %

            
    %         else
    %             % -------------------------- Ongoing Activity State -------------------------- %
    %             robot{i}.ongoing_activity.completed = 1;
    %             % ---------------------------------------------------------------------------- %

    %             % --------------------------------- Dismissal -------------------------------- %
    %             robot{i}.completed_activities{end + 1} = robot{i}.ongoing_activity.activity;
    %             % ---------------------------------------------------------------------------- %

    %             % ----------------------------------- Reset ---------------------------------- %
    %             robot{i}.ongoing_activity.activity = {};
    %             robot{i}.ongoing_activity.ongoing_phase = 1;
    %             robot{i}.ongoing_activity.completed = 0;
    %             % ---------------------------------------------------------------------------- %
    %         end
    %     end
    %     % ---------------------------------------------------------------------------- %

    %     % ---------------------------------------------------------------------------- %


    %     % ---------------------------------------------------------------------------- %
    %     %                         Allocation of a New Activity                         %
    %     % ---------------------------------------------------------------------------- %
    %     elseif (length(robot{i}.activities) > 0) && (robot{i}.activities{1}.start_time >= time.elapsed_time)
    %         % ----------------------------- Goal Construction ---------------------------- %            
    %         goal = build_goal(robot{i}.activities{1}.phase{1}.type,spline_interpolation,robot{i}.controller.b, ...
    %                           robot{i}.pose,0.0,robot{i}.activities{1}.phase{1}.desired_configuration, ...
    %                           robot{i}.activities{1}.phase{1}.allocated_time,time.elapsed_time);                  
    %         % ---------------------------------------------------------------------------- %

    %         % -------------------------------- Allocation -------------------------------- %
    %         robot{i}.ongoing_activity.activity = robot{i}.activities{1};
    %         robot{i}.ongoing_activity.activity.phase{1}.goal = goal;
    %         % ---------------------------------------------------------------------------- %

    %         % ------------------- Updating of the Activities Structure ------------------- %
    %         if (length(robot{i}.activities) - 1) > 0
    %             robot{i}.activities = robot{i}.activities{2:end};
    %         else
    %             robot{i}.activities = {};
    %         end
    %         % ---------------------------------------------------------------------------- %

    %         % -------------------------- Reset of the Parameters ------------------------- %
    %         robot{i}.ongoing_activity.ongoing_phase = 1;
    %         robot{i}.ongoing_activity.completed = 0;
    %         % ---------------------------------------------------------------------------- %

    %     % ---------------------------------------------------------------------------- %

    %     % ---------------------------------------------------------------------------- %
    %     %                                 Nothing to do                                %
    %     % ---------------------------------------------------------------------------- %
    %     else
    %         % -------------------------------- Velocities -------------------------------- %
    %         robot{i}.linearVelocity = 0.0;
    %         robot{i}.angularVelocity = 0.0;
    %         % ---------------------------------------------------------------------------- %
    %     end
    %     % ---------------------------------------------------------------------------- %
    % end
    % % ---------------------------------------------------------------------------- %

    

    %     % -------------------------- Experiment - Start Time ------------------------- %
    %     if robot{i}.start_time.global.initialized == 0
    %         reset(r);
    %         robot{i}.start_time.global.value = r.TotalElapsedTime;
    %         robot{i}.start_time.global.initialized = 1;
    %     end
    %     % ---------------------------------------------------------------------------- %

    %     % ----------------------------------- Pose ----------------------------------- %
    %     robot{i}.odometry.Data = robot{i}.odometry.Subscriber.LatestMessage;
    %     robot{i}.pose = odometry_robot_pose_sim(robot{i}.odometry.Data.Pose);

    %     robot{i}.controller.y_1 = robot{i}.pose(1,1) + (robot{i}.controller.b * cos(robot{i}.pose(1,3)));
    %     robot{i}.controller.y_2 = robot{i}.pose(1,2) + (robot{i}.controller.b * sin(robot{i}.pose(1,3)));

    %     robot{i}.controller.error = robot{i}.goal.final_position - [robot{i}.controller.y_1, robot{i}.controller.y_2];
    %     % ---------------------------------------------------------------------------- %

    %     % ---------------------------------- Spline ---------------------------------- %
    %     elapsed_time = r.TotalElapsedTime - robot{i}.start_time.global.value;
    %     [traj,dtraj] = spline_trajectory(robot{i}.goal.initial_position(1,1:2), ... 
    %                                      robot{i}.goal.corner_position(1,1:2), ...
    %                                      robot{i}.goal.final_position(1,1:2), ...
    %                                      robot{i}.goal.instants(1,1), ...
    %                                      robot{i}.goal.instants(1,2), ...
    %                                      robot{i}.goal.instants(1,3), ...
    %                                      elapsed_time);
    %     % ---------------------------------------------------------------------------- %

    %     % ---------------------------------------------------------------------------- %

    %     % ---------------------------------------------------------------------------- %
    %     %                          Input/Output Linearization                          %
    %     % ---------------------------------------------------------------------------- %
    %     robot{i}.controller.y_1d = traj(1,1);
    %     robot{i}.controller.y_2d = traj(1,2);
        
    %     robot{i}.controller.dy_1d = dtraj(1,1);
    %     robot{i}.controller.dy_2d = dtraj(1,2);

    %     u_1 = robot{i}.controller.dy_1d + robot{i}.controller.k1*(robot{i}.controller.y_1d - robot{i}.controller.y_1);
    %     u_2 = robot{i}.controller.dy_2d + robot{i}.controller.k2*(robot{i}.controller.y_2d - robot{i}.controller.y_2);
    %     % ---------------------------------------------------------------------------- %

    %     % ---------------------------------------------------------------------------- %
    %     %                              Obstacle Avoidance                              %
    %     % ---------------------------------------------------------------------------- %
    %     if obstacleAvoidanceEnabled == true
    %         for j=1:num_robots
    %             if j~=i
    %                 distance_ij = norm([robot{j}.controller.y_1, robot{j}.controller.y_2] - [robot{i}.controller.y_1, robot{i}.controller.y_2]);
    %                 r_ij = ([robot{j}.controller.y_1, robot{j}.controller.y_2] - [robot{i}.controller.y_1, robot{i}.controller.y_2])/distance_ij;

                    
    %                 if distance_ij <= obstacleAvoidanceThreshold
    %                     linearVelocityOffset = (-r_ij)*(obstacleAvoidanceGain/(distance_ij^2))*(1/distance_ij - 1/obstacleAvoidanceThreshold)^(obstacleAvoidanceEta - 1);
    %                     robot{i}.controller.obstacleAvoidance.state = true;
    %                 else
    %                     robot{i}.controller.obstacleAvoidance.state = false;
    %                     linearVelocityOffset = [0, 0];
    %                 end

    %                 u_1 = u_1 + linearVelocityOffset(1,1);
    %                 u_2 = u_2 + linearVelocityOffset(1,2);
    %             end
    %         end
    %     end
    %     % ---------------------------------------------------------------------------- %

    %     % ---------------------------------------------------------------------------- %
    %     %                                  Velocities                                  %
    %     % ---------------------------------------------------------------------------- %
    %     velocities = [cos(robot{i}.pose(1,3)) sin(robot{i}.pose(1,3)); -(sin(robot{i}.pose(1,3))/robot{i}.controller.b) (cos(robot{i}.pose(1,3))/robot{i}.controller.b)]*[u_1 u_2]';

    %     robot{i}.linearVelocity = velocities(1,1);
    %     robot{i}.angularVelocity = velocities(2,1);
    %     robot{i}.linearVelocity = sign(robot{i}.linearVelocity)*min(abs(robot{i}.linearVelocity),0.25);
    %     robot{i}.angularVelocity = sign(robot{i}.angularVelocity)*min(abs(robot{i}.angularVelocity),1.0);
    %     % ---------------------------------------------------------------------------- %

    %     % ---------------------------------------------------------------------------- %
    %     %                                     State                                    %
    %     % ---------------------------------------------------------------------------- %
    %     robot{i}.q.nominal(end+1,:) = [robot{i}.controller.y_1d robot{i}.controller.y_2d];
    %     robot{i}.q.actual(end+1,:) = [robot{i}.controller.y_1 robot{i}.controller.y_2];

    %     robot{i}.dq.nominal(end+1,:) = [robot{i}.controller.dy_1d robot{i}.controller.dy_2d];
    %     robot{i}.dq.actual(end+1,:) = [u_1, u_2];


    %     robot{i}.time(end+1,1) = loop_rate.TotalElapsedTime - robot{i}.start_time.global.value;
    %     % ---------------------------------------------------------------------------- %

    %     % ----------------------------------- Check ---------------------------------- %
    %     if (norm(robot{i}.controller.error) <= robot{i}.controller.distance_threshold)
    %         disp(['Robot ', num2str(i), ' reached the final waypoint. Stopping.']);
    %         robot{i}.goal.final_waypoint_reached = 1;

    %         robot{i}.linearVelocity = 0.0;
    %         robot{i}.angularVelocity = 0.0;
    %     end
    %     % ---------------------------------------------------------------------------- %

    % end
    % % ---------------------------------------------------------------------------- %
    
    % % ------------------------------- ROS Messages ------------------------------- %
    % for i=1:num_robots
    %     vel_msg = rosmessage(robot{i}.velocities_publisher);
    %     vel_msg.Linear.X = robot{i}.linearVelocity;
    %     vel_msg.Angular.Z = robot{i}.angularVelocity;
    %     send(robot{i}.velocities_publisher, vel_msg);

    %     % if robot{i}.controller.obstacleAvoidance.state == false
    %     %     plot(robot{i}.pose(1), robot{i}.pose(2), '-*', 'Color', "#77AC30");
    %     % else
    %     %     plot(robot{i}.pose(1), robot{i}.pose(2), '-*', 'Color', "#D95319");
    %     % end
    %     % drawnow;
    % end
    % % ---------------------------------------------------------------------------- %

    % % ----------------------------------- Check ---------------------------------- %
    % for i=1:num_robots
    %     if robot{i}.goal.final_waypoint_reached == 0
    %         break;
    %     elseif i == num_robots
    %         robots_plots(robot,num_robots);
    %         rosshutdown;
    %     end
    % end
    % % ---------------------------------------------------------------------------- %
end
% ---------------------------------------------------------------------------- %

% ---------------------------------------------------------------------------- %
%                                   FUNCTIONS                                  %
% ---------------------------------------------------------------------------- %
function [] = robots_parameters_and_controllers_initialization(simulation,b,distanceThreshold,home_positions)
    % ---------------------------------------------------------------------------- %
    %                          Global and Local Variables                          %
    % ---------------------------------------------------------------------------- %
    global gazeboModelStates;
    global num_robots;
    global robot;
    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %
    %                                  Processing                                  %
    % ---------------------------------------------------------------------------- %
    for i=1:num_robots
        % ------------------------ Personal Information: Name ------------------------ %
        robot{i}.name = sprintf('diff_drive_robot%d', i);
        % ---------------------------------------------------------------------------- %

        % ---------------------------- Simulation Mode: ON --------------------------- %
        if simulation == 1
            % ------------------------- Personal Information: ID ------------------------- %
            robot{i}.ID = find(strcmp(gazeboModelStates.Data.Name, robot{i}.name)); % returns the linear indices corresponding to the nonzero entries of the array
            % ---------------------------------------------------------------------------- %

            % ------------------------------- Current Pose ------------------------------- %
            robot{i}.pose = odometry_robot_pose_sim(robot{i}.odometry.Data.Pose);
            % ---------------------------------------------------------------------------- %
        % ---------------------------------------------------------------------------- %
        % --------------------------- Simulation Mode: OFF --------------------------- %
        else
            % ------------------------- Personal Information: ID ------------------------- %
            robot{i}.ID = i;
            % ---------------------------------------------------------------------------- %

            % ------------------------------- Current Pose ------------------------------- %
            robot{i}.pose = odometry_robot_pose_real(robot{i}.odometry.Data);
            % ---------------------------------------------------------------------------- %
        end
        % ---------------------------------------------------------------------------- %

        % ---------------------------- Home Configuration ---------------------------- %
        robot{i}.home_position = home_positions(i,:);
        % ---------------------------------------------------------------------------- %

        % -------------------------------- Controller -------------------------------- %
        robot{i}.controller.b = b;  % Control point

        robot{i}.controller.y_1 = robot{i}.pose(1,1) + (robot{i}.controller.b * cos(robot{i}.pose(1,3)));
        robot{i}.controller.y_2 = robot{i}.pose(1,2) + (robot{i}.controller.b * sin(robot{i}.pose(1,3)));

        robot{i}.controller.dy_1 = 0.0;
        robot{i}.controller.dy_2 = 0.0;

        robot{i}.controller.y_1d = robot{i}.controller.y_1;
        robot{i}.controller.y_2d = robot{i}.controller.y_2;

        robot{i}.controller.dy_1d = robot{i}.controller.dy_1;
        robot{i}.controller.dy_2d = robot{i}.controller.dy_1;

        robot{i}.controller.error = 0.0;

        robot{i}.controller.distance_threshold = distanceThreshold;

        robot{i}.controller.k1 = 0.8; % 1.5; % First Gain
        robot{i}.controller.k2 = 0.8; % 1.5; % Second Gain
        robot{i}.controller.k3 = 1.0; % Second Gain
        % ---------------------------------------------------------------------------- %

        % ----------------------------------- State ---------------------------------- %
        robot{i}.time = 0.0;
        robot{i}.q.nominal = [robot{i}.controller.y_1d robot{i}.controller.y_2d];
        robot{i}.q.actual = [robot{i}.controller.y_1 robot{i}.controller.y_2];

        robot{i}.dq.nominal = [robot{i}.controller.dy_1d robot{i}.controller.dy_2d];
        robot{i}.dq.actual = [robot{i}.controller.dy_1 robot{i}.controller.dy_2];
        % ---------------------------------------------------------------------------- %

        % ----------------------------- Other Parameters ----------------------------- %
        robot{i}.activities = {};

        robot{i}.ongoing_activity.ongoing_phase = 1;
        robot{i}.ongoing_activity.activity = {};
        robot{i}.ongoing_activity.completed = 1;

        robot{i}.completed_activities = {};
        % ---------------------------------------------------------------------------- %

    end
    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %
end

function [] = publishers_and_subscribers_definition(simulation)
    % ---------------------------------------------------------------------------- %
    %                          Global and Local Variables                          %
    % ---------------------------------------------------------------------------- %
    global gazeboModelStates;
    global num_robots;
    global robot;
    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %
    %                                  Processing                                  %
    % ---------------------------------------------------------------------------- %
    for i=1:num_robots
        % ---------------------------- Simulation Mode: ON --------------------------- %
        if simulation == 1
            robot{i}.velocities_publisher = rospublisher(sprintf('/robot%d/cmd_vel', i),'geometry_msgs/Twist');
            robot{i}.odometry.Subscriber = rossubscriber(sprintf('/robot%d/odom', i), 'nav_msgs/Odometry');
        % ---------------------------------------------------------------------------- %
        % --------------------------- Simulation Mode: OFF --------------------------- %
        else
            if i == 1
                robot{i}.velocities_publisher = rospublisher('/nuc1/cmd_vel_mux/input/teleop','geometry_msgs/Twist');
                robot{i}.odometry.Subscriber = rossubscriber('/vrpn_client_node/Turtlebot1/pose','geometry_msgs/PoseStamped');
            elseif i == 2
                robot{i}.velocities_publisher = rospublisher('/nuc3/cmd_vel_mux/input/teleop','geometry_msgs/Twist');
                robot{i}.odometry.Subscriber = rossubscriber('/vrpn_client_node/Turtlebot3/pose','geometry_msgs/PoseStamped');
            end
        end
        % ---------------------------------------------------------------------------- %

        % --------------------------------- Odometry --------------------------------- %
        robot{i}.odometry.Data = receive(robot{i}.odometry.Subscriber,1);
        % ---------------------------------------------------------------------------- %
    end

    % ------------------------------- Gazebo State ------------------------------- %
    if simulation == 1
        gazeboModelStates.Subscriber = rossubscriber('/gazebo/model_states', 'gazebo_msgs/ModelStates');
        gazeboModelStates.Data = receive(gazeboModelStates.Subscriber);
    end
    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %
end

function [] = plan_processing()
    % ---------------------------------------------------------------------------- %
    %                          Global and Local Variables                          %
    % ---------------------------------------------------------------------------- %
    global plan;
    global num_robots;
    global robot;
    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %
    %                                   Fake Plan                                  %
    % ---------------------------------------------------------------------------- %
    plan.humans{1}.position = [1.3, 1.8];
    plan.humans{2}.position = [0.8, 2.0];

    plan.robots{1}.activities{1}.humand_ID = 1;
    plan.robots{1}.activities{1}.going_phase = 40;
    plan.robots{1}.activities{1}.waiting_phase = 5;
    plan.robots{1}.activities{1}.serving_phase = 15;
    plan.robots{1}.activities{1}.depositing_phase = 40;
    
    plan.robots{2}.activities{1}.humand_ID = 2;
    plan.robots{2}.activities{1}.going_phase = 40;
    plan.robots{2}.activities{1}.waiting_phase = 7;
    plan.robots{2}.activities{1}.serving_phase = 15;
    plan.robots{2}.activities{1}.depositing_phase = 40;
    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %
    %                             Robot Data Structure                             %
    % ---------------------------------------------------------------------------- %
    for i=1:num_robots
        % -------------------------------- Activities -------------------------------- %
        for j=1:length(plan.robots{i}.activities)

            robot{i}.activities{j}.start_time = 0.0;

            robot{i}.activities{j}.phase{1}.type = 0; % 0 - Moving phase (Forward Motion), 1 - Moving phase (Backward Motion), -1 - Waiting phase
            robot{i}.activities{j}.phase{1}.allocated_time = plan.robots{i}.activities{j}.going_phase;
            robot{i}.activities{j}.phase{1}.desired_configuration = [plan.humans{plan.robots{i}.activities{j}.humand_ID}.position, 0.0];
            robot{i}.activities{j}.phase{1}.goal = {};

            robot{i}.activities{j}.phase{2}.type = -1; % 0 - Moving phase (Forward Motion), 1 - Moving phase (Backward Motion), -1 - Waiting phase
            robot{i}.activities{j}.phase{2}.allocated_time = plan.robots{i}.activities{j}.waiting_phase;
            robot{i}.activities{j}.phase{2}.desired_configuration = [];
            robot{i}.activities{j}.phase{2}.goal = {};

            robot{i}.activities{j}.phase{3}.type = -1; % 0 - Moving phase (Forward Motion), 1 - Moving phase (Backward Motion), -1 - Waiting phase
            robot{i}.activities{j}.phase{3}.allocated_time = plan.robots{i}.activities{j}.serving_phase;
            robot{i}.activities{j}.phase{3}.desired_configuration = [];
            robot{i}.activities{j}.phase{3}.goal = {};

            robot{i}.activities{j}.phase{4}.type = 1; % 0 - Moving phase (Forward Motion), 1 - Moving phase (Backward Motion), -1 - Waiting phase
            robot{i}.activities{j}.phase{4}.allocated_time = plan.robots{i}.activities{j}.depositing_phase;
            robot{i}.activities{j}.phase{4}.desired_configuration = [robot{i}.home_position, -pi];
            robot{i}.activities{j}.phase{4}.goal = {};
        end
        % ---------------------------------------------------------------------------- %
    end
    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %
end

function goal = build_goal(goal_type,spline_interpolation,b,initial_pose,mid_pose,final_pose,allocated_time,t)
    % ---------------------------------------------------------------------------- %
    %                          Global and Local Variables                          %
    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %
    %                                  Processing                                  %
    % ---------------------------------------------------------------------------- %
    
    % ----------------------------------- Type ----------------------------------- %
    goal.type = goal_type; % 0 - Moving phase (Forward Motion), 1 - Moving phase (Backward Motion), -1 - Waiting phase
    % ---------------------------------------------------------------------------- %

    % ------------------------------- Construction ------------------------------- %
    if goal_type ~= -1
        if spline_interpolation == 1
            goal.initial_orientation = initial_pose(1,3);
            goal.initial_position(1,1) = initial_pose(1,1) + (b * cos(initial_pose(1,3)));
            goal.initial_position(1,2) = initial_pose(1,2) + (b * sin(initial_pose(1,3)));

            goal.corner_orientation =  (final_pose(1,3) - initial_pose(1,3))/2.0;

            if goal_type == 0
                goal.corner_position =  [final_pose(1,1) goal.initial_position(1,2)];
            elseif goal_type == 1
                goal.corner_position =  [goal.initial_position(1,1) final_pose(1,2)];
            end
            
            goal.corner_position(1,1) = goal.corner_position(1,1) + (b * cos(goal.corner_orientation));
            goal.corner_position(1,2) = goal.corner_position(1,2) + (b * sin(goal.corner_orientation));
            
            goal.final_orientation = final_pose(1,3);
            goal.final_position = final_pose(1,1:2);
            goal.final_position(1,1) = goal.final_position(1,1) + (b * cos(goal.final_orientation));
            goal.final_position(1,2) = goal.final_position(1,2) + (b * sin(goal.final_orientation));

            d_ic = norm(goal.corner_position - goal.initial_position);
            d_cf = norm(goal.final_position - goal.corner_position);
            d = d_ic + d_cf;

            t_i = t;
            t_c = t_i + allocated_time*(d_ic/d);
            t_f = t_c + allocated_time*(d_cf/d);

            goal.displacement = goal.final_position - goal.initial_position;
            goal.instants = [t_i,t_c,t_f];
            goal.completed = 0; % 0 - false, 1 - true
        end
    else
        goal.initial_orientation = initial_pose(1,3);
        goal.initial_position(1,1) = initial_pose(1,1) + (b * cos(initial_pose(1,3)));
        goal.initial_position(1,2) = initial_pose(1,2) + (b * sin(initial_pose(1,3)));

        goal.corner_orientation =  [];
        goal.corner_position =  [];

        goal.final_orientation = goal.initial_orientation;
        goal.final_position = goal.initial_position;

        t_i = t;
        t_f = t_i + allocated_time;

        goal.displacement = goal.final_position - goal.initial_position;
        goal.instants = [t_i,t_f];
        goal.completed = 0; % 0 - false, 1 - true
    end
    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %
end

function gazebo_robot_pose = gazebo_robot_pose(gazeboModelStates,index)
    % ---------------------------------------------------------------------------- %
    %                                  Processing                                  %
    % ---------------------------------------------------------------------------- %

    % --------------------------------- Position --------------------------------- %
    x = gazeboModelStates.Pose(index).Position.X;
    y = gazeboModelStates.Pose(index).Position.Y;
    % ---------------------------------------------------------------------------- %
    

    % -------------------------- Orientation: Quaternion ------------------------- %
    Qx = gazeboModelStates.Pose(index).Orientation.X;
    Qy = gazeboModelStates.Pose(index).Orientation.Y;
    Qz = gazeboModelStates.Pose(index).Orientation.Z;
    Qw = gazeboModelStates.Pose(index).Orientation.W;
    % ---------------------------------------------------------------------------- %

    % ---------------------------- Orientation: Euler ---------------------------- %
    RPY = quat2eul([Qw,Qx,Qy,Qz],'XYZ');
    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %
    %                                    Return                                    %
    % ---------------------------------------------------------------------------- %
    gazebo_robot_pose = [x,y,RPY(1,3)];
    % ---------------------------------------------------------------------------- %
end

function odometry_robot_pose_sim = odometry_robot_pose_sim(navMsg)
    % ---------------------------------------------------------------------------- %
    %                                  Processing                                  %
    % ---------------------------------------------------------------------------- %

    % --------------------------------- Position --------------------------------- %
    x = navMsg.Pose.Position.X;
    y = navMsg.Pose.Position.Y;
    % ---------------------------------------------------------------------------- %
    

    % -------------------------- Orientation: Quaternion ------------------------- %
    Qx = navMsg.Pose.Orientation.X;
    Qy = navMsg.Pose.Orientation.Y;
    Qz = navMsg.Pose.Orientation.Z;
    Qw = navMsg.Pose.Orientation.W;
    % ---------------------------------------------------------------------------- %

    % ---------------------------- Orientation: Euler ---------------------------- %
    RPY = quat2eul([Qw,Qx,Qy,Qz],'XYZ');
    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %
    %                                    Return                                    %
    % ---------------------------------------------------------------------------- %
    odometry_robot_pose_sim = [x,y,RPY(1,3)];
    % ---------------------------------------------------------------------------- %
end

function odometry_robot_pose_real = odometry_robot_pose_real(poseStampedMsg)
    % ---------------------------------------------------------------------------- %
    %                                  Processing                                  %
    % ---------------------------------------------------------------------------- %

    % --------------------------------- Position --------------------------------- %
    x = poseStampedMsg.Pose.Position.X;
    y = poseStampedMsg.Pose.Position.Y;
    % ---------------------------------------------------------------------------- %
    

    % -------------------------- Orientation: Quaternion ------------------------- %
    Qx = poseStampedMsg.Pose.Orientation.X;
    Qy = poseStampedMsg.Pose.Orientation.Y;
    Qz = poseStampedMsg.Pose.Orientation.Z;
    Qw = poseStampedMsg.Pose.Orientation.W;
    % ---------------------------------------------------------------------------- %

    % ---------------------------- Orientation: Euler ---------------------------- %
    RPY = quat2eul([Qw,Qx,Qy,Qz],'XYZ');
    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %
    %                                    Return                                    %
    % ---------------------------------------------------------------------------- %
    odometry_robot_pose_real = [x,y,RPY(1,3)];
    % ---------------------------------------------------------------------------- %
end

function orientation_error = orientation_error(ang_des,ang_real)
    if ang_des < 0
        ang_des = ang_des + 2*pi;
    elseif ang_des > 2*pi
        ang_des = ang_des - 2*pi;
    end

    if ang_real < 0
        ang_real = ang_real + ceil(abs(ang_real)/(2*pi))*2*pi;
    elseif ang_real > 2*pi
        ang_real = ang_real - ceil(abs(ang_real)/(2*pi))*2*pi;
    end

    orientation_error = ang_des - ang_real;

    ang_des
    ang_real
end
% ---------------------------------------------------------------------------- %

