% ---------------------------------------------------------------------------- %
%                                     Reset                                    %
% ---------------------------------------------------------------------------- %
rosshutdown;
close all;
clc;
clear;
clear global;
simulation = 0;
nuc1nuc3 = 1;
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

% ----------------------------- Global Parameters ---------------------------- %
global num_robots;
num_robots = 2;
% ---------------------------------------------------------------------------- %

% ------------------------------------ ROS ----------------------------------- %
if nuc1nuc3 == 0
    for i=1:num_robots
        if simulation == 1
            robot{i}.velocities_publisher = rospublisher(sprintf('/robot%d/cmd_vel', i),'geometry_msgs/Twist');
            robot{i}.odometry.Subscriber = rossubscriber(sprintf('/robot%d/odom', i), 'nav_msgs/Odometry');
        else
            robot{i}.velocities_publisher = rospublisher(sprintf('/nuc%d/cmd_vel_mux/input/teleop', i),'geometry_msgs/Twist');
            robot{i}.odometry.Subscriber = rossubscriber(sprintf('/vrpn_client_node/Turtlebot%d/pose', i), 'geometry_msgs/PoseStamped');
        end
        robot{i}.odometry.Data = receive(robot{i}.odometry.Subscriber,1);
    end
else
    robot{1}.velocities_publisher = rospublisher('/nuc1/cmd_vel_mux/input/teleop','geometry_msgs/Twist');
    robot{1}.odometry.Subscriber = rossubscriber('/vrpn_client_node/Turtlebot1/pose', 'geometry_msgs/PoseStamped');
    robot{1}.odometry.Data = receive(robot{1}.odometry.Subscriber,1);

    robot{2}.velocities_publisher = rospublisher('/nuc3/cmd_vel_mux/input/teleop','geometry_msgs/Twist');
    robot{2}.odometry.Subscriber = rossubscriber('/vrpn_client_node/Turtlebot3/pose', 'geometry_msgs/PoseStamped');
    robot{2}.odometry.Data = receive(robot{2}.odometry.Subscriber,2);
end

if simulation == 1
    gazeboModelStates.Subscriber = rossubscriber('/gazebo/model_states', 'gazebo_msgs/ModelStates');
    gazeboModelStates.Data = receive(gazeboModelStates.Subscriber);
end
% ---------------------------------------------------------------------------- %

% ----------------------------------- Robot ---------------------------------- %
for i=1:num_robots
    robot{i}.name = sprintf('diff_drive_robot%d', i);
    if simulation == 1
        robot{i}.index = find(strcmp(gazeboModelStates.Data.Name, robot{i}.name)); % returns the linear indices corresponding to the nonzero entries of the array
    else
        robot{i}.index = i;
    end
end

distanceThreshold = 0.1;
% ---------------------------------------------------------------------------- %

% ----------------------------------- Path ----------------------------------- %
warehouse_positions = [-0.9, 0.1; -1, 0.8]; % Initial Location
warehouse_orientation = 0.0; % Initial Orientation of the Robot (The robot orientation is the angle between the robot heading and the positive X-axis, measured counterclockwise).

desired_positions = [1.3, 1.8; 0.8, 2.0];
%desired_positions = [2.4, 3.8; 2.0, 3.0];


desired_timing = [30; 50];

for i=1:num_robots
    robot{i}.home_position = warehouse_positions(i,:);
    robot{i}.waypoints.positions = [desired_positions(i,1), robot{i}.home_position(1,2); desired_positions(i,:); desired_positions(i,1), robot{i}.home_position(1,2);warehouse_positions(i,:)];
    robot{i}.waypoints.orientations = [0; pi/2.0; -pi/2.0; pi]
    robot{i}.waypoints.times = [20.0;20.0;20.0;20.0];

    robot{i}.goal.initial_position =  warehouse_positions(i,:);
    robot{i}.goal.initial_orientation = 0;
    robot{i}.goal.final_position = robot{i}.waypoints.positions(1,:);
    robot{i}.goal.final_orientation = 0;
    robot{i}.goal.displacement = robot{i}.goal.final_position - robot{i}.goal.initial_position;
    robot{i}.goal.duration = robot{i}.waypoints.times(1,1);
    robot{i}.goal.final_waypoint_reached = 0;

    robot{i}.start_time.local.updated = 0;
    robot{i}.start_time.local.value = 0;

    robot{i}.start_time.global.initialized = 0;
    robot{i}.start_time.global.value = 0;
end
% ---------------------------------------------------------------------------- %

% ----------------------------------- Time ----------------------------------- %
frequency =  100;
Ts = 1/frequency;
loop_rate = rateControl(frequency);
r = rosrate(frequency);
elapsed_time = 0;
% ---------------------------------------------------------------------------- %

% ----------------------------------- Task ----------------------------------- %
for i=1:num_robots
    robot{i}.controller.pick_phase.completed = 0;
    robot{i}.controller.pick_phase.waypoint_index = 1;

    robot{i}.controller.depot_phase.completed = 0;
    robot{i}.controller.depot_phase.waypoint_index = size(robot{i}.waypoints.positions,1);
end
% ---------------------------------------------------------------------------- %

% -------------------------------- Controller -------------------------------- %
for i=1:num_robots
    robot{i}.controller.waypoint_index = 1;

    robot{i}.controller.b = 0.06;  % Control point

    robot{i}.controller.k1 = 1.0; % First Gain
    robot{i}.controller.k2 = 1; % Second Gain
    robot{i}.controller.k3 = 1; % Second Gain
end
% ---------------------------------------------------------------------------- %


% ----------------------------------- Pose ----------------------------------- %
for i=1:num_robots
    if simulation == 1
        robot{i}.pose = odometry_robot_pose_sim(robot{i}.odometry.Data.Pose);
    else
        robot{i}.pose = odometry_robot_pose_real(robot{i}.odometry.Data);
    end
end
% ---------------------------------------------------------------------------- %

% ----------------------------------- State ---------------------------------- %
for i=1:num_robots
    robot{i}.time = 0.0;
    robot{i}.q.nominal = robot{i}.pose(1,1:2);
    robot{i}.q.actual = robot{i}.pose(1,1:2);
    robot{i}.G = [cos(robot{i}.pose(1,3)) 0; sin(robot{i}.pose(1,3)) 0; 0 1];
end
% ---------------------------------------------------------------------------- %

% ---------------------------------- Figure ---------------------------------- %
figure
title('Planned L-shaped Trajectory');
grid on;
hold on;
xlabel('X (m)');
ylabel('Y (m)');

for i=1:num_robots
    plot(robot{i}.waypoints.positions(:,1), robot{i}.waypoints.positions(:,2), '-o', 'LineWidth', 2);
    xlim([min([warehouse_positions(:,1),desired_positions(:,1)],[],'all') - 1, max([warehouse_positions(:,1),desired_positions(:,1)],[],'all') + 1]);
    ylim([min([warehouse_positions(:,2),desired_positions(:,2)],[],'all') - 1, max([warehouse_positions(:,2),desired_positions(:,2)],[],'all') + 1]);
end
% ---------------------------------------------------------------------------- %

% ---------------------------------------------------------------------------- %

% ---------------------------------------------------------------------------- %
%                                   Main Loop                                  %
% ---------------------------------------------------------------------------- %
while ros.internal.Global.isNodeActive
    % -------------------------------- i-th Robot -------------------------------- %
    for i=1:num_robots


        % -------------------------- Experiment - Start Time ------------------------- %
        if robot{i}.start_time.global.initialized == 0
            reset(r);
            robot{i}.start_time.global.value = r.TotalElapsedTime;
            robot{i}.start_time.global.initialized = 1;
        end
        % ---------------------------------------------------------------------------- %

        % -------------------------- Local Goal - Start Time ------------------------- %
        if robot{i}.start_time.local.updated == 0
            robot{i}.start_time.local.value = r.TotalElapsedTime;
            robot{i}.start_time.local.updated = 1;
        end
        % ---------------------------------------------------------------------------- %

        % ----------------------------------- Pose ----------------------------------- %
        if simulation == 1
            robot{i}.odometry.Data = robot{i}.odometry.Subscriber.LatestMessage;
            robot{i}.pose = odometry_robot_pose_sim(robot{i}.odometry.Data.Pose);
        else
            robot{i}.odometry.Data = receive(robot{i}.odometry.Subscriber,1);
            robot{i}.pose = odometry_robot_pose_real(robot{i}.odometry.Data);
        end
        % ---------------------------------------------------------------------------- %

        % ----------------------- Trapezoidal Velocity Profile ----------------------- %
        elapsed_time = r.TotalElapsedTime - robot{i}.start_time.local.value
        [s,ds,dds] = trapezoidal(0,1,elapsed_time,robot{i}.goal.duration);
        % ---------------------------------------------------------------------------- %

        % ---------------------------------------------------------------------------- %
        %                              Non-Linear Control                              %
        % ---------------------------------------------------------------------------- %

        % % ---------------------------- Control Parameters ---------------------------- %
        desired_position = robot{i}.goal.initial_position + (robot{i}.goal.final_position - robot{i}.goal.initial_position)*s;
        % desired_velocity = (robot{i}.goal.final_position - robot{i}.goal.initial_position)*ds;

        % theta_des = atan2(robot{i}.goal.displacement(1,2),robot{i}.goal.displacement(1,1));
        % theta = robot{i}.pose(1,3);

        % pos_error = desired_position - robot{i}.pose(1,1:2);
        % e1 = (cos(theta)*pos_error(1,1)+ sin(theta)*pos_error(1,2));
        % e2 = (-sin(theta)*pos_error(1,1)+ cos(theta)*pos_error(1,2));
        % e3= -angdiff(theta, theta_des);

        % if(abs(e3)<10^-5)
        %     sin_e3 = 0;
        % else
        %     sin_e3 = sin(e3)/(e3);
        % end
        % % ---------------------------------------------------------------------------- %

        % % ------------------------ Linear and Angular Velocity ----------------------- %
        % robot{i}.linearVelocity = (ds*cos(e3)) + (e1*robot{i}.controller.k1);
        
        % robot{i}.angularVelocity = 0 + (ds*sin_e3*e2*robot{i}.controller.k2) - (e3*robot{i}.controller.k3);

        % robot{i}.linearVelocity = sign(robot{i}.linearVelocity)*min(abs(robot{i}.linearVelocity),0.25);
        % robot{i}.angularVelocity = sign(robot{i}.angularVelocity)*min(abs(robot{i}.angularVelocity),1.0);
        % % ---------------------------------------------------------------------------- %

        % ---------------------------------------------------------------------------- %

        % ---------------------------------------------------------------------------- %
        %                          Input/Output Linearization                          %
        % ---------------------------------------------------------------------------- %
        y_1_i = robot{i}.goal.initial_position(1,1) + (robot{i}.controller.b * cos(robot{i}.goal.initial_orientation));
        y_2_i = robot{i}.goal.initial_position(1,2) + (robot{i}.controller.b * cos(robot{i}.goal.initial_orientation));
        
        y_1_f = robot{i}.goal.final_position(1,1) + (robot{i}.controller.b * cos(robot{i}.goal.final_orientation));
        y_2_f = robot{i}.goal.final_position(1,2) + (robot{i}.controller.b * cos(robot{i}.goal.final_orientation));

        y_1d = y_1_i + (y_1_f - y_1_i)*s;
        y_2d = y_2_i + (y_2_f - y_2_i)*s;

        dy_1d = (y_1_f - y_1_i)*ds;
        dy_2d = (y_2_f - y_2_i)*ds;
        
        displacement = [y_1_f, y_2_f] - [y_1_i, y_2_i];
        theta_des = atan2(displacement(1,2),displacement(1,1));

        y_1 = robot{i}.pose(1,1) + (robot{i}.controller.b * cos(robot{i}.pose(1,3)));
        y_2 = robot{i}.pose(1,2) + (robot{i}.controller.b * sin(robot{i}.pose(1,3)));


        u_1 = dy_1d + 1.5*(y_1d - y_1);
        u_2 = dy_2d + 1.5*(y_2d - y_2);
        
        velocities = [cos(robot{i}.pose(1,3)) sin(robot{i}.pose(1,3)); -(sin(robot{i}.pose(1,3))/robot{i}.controller.b) (cos(robot{i}.pose(1,3))/robot{i}.controller.b)]*[u_1 u_2]';

        robot{i}.linearVelocity = velocities(1,1);
        robot{i}.angularVelocity = velocities(2,1);
        robot{i}.linearVelocity = sign(robot{i}.linearVelocity)*min(abs(robot{i}.linearVelocity),0.25);
        robot{i}.angularVelocity = sign(robot{i}.angularVelocity)*min(abs(robot{i}.angularVelocity),1.0);
        % ---------------------------------------------------------------------------- %

        % ----------------------------------- State ---------------------------------- %
        robot{i}.q.nominal(end+1,:) = desired_position;
        robot{i}.q.actual(end+1,:) = robot{i}.pose(1,1:2);

        robot{i}.time(end+1,1) = loop_rate.TotalElapsedTime - robot{i}.start_time.global.value;
        % ---------------------------------------------------------------------------- %

        % ----------------------------------- Check ---------------------------------- %
        if elapsed_time >= (robot{i}.goal.duration + 5)
            % -------------------------------- Velocities -------------------------------- %
            robot{i}.linearVelocity = 0.0;
            robot{i}.angularVelocity = 0.0;
            % ---------------------------------------------------------------------------- %

            % --------------------------------- Waypoint --------------------------------- %
            if robot{i}.controller.waypoint_index < size(robot{i}.waypoints.positions,1)
                % ----------------------------------- Time ----------------------------------- %
                robot{i}.start_time.local.updated = 0;
                % ---------------------------------------------------------------------------- %

                % ----------------------------------- Goal ----------------------------------- %
                robot{i}.controller.waypoint_index = robot{i}.controller.waypoint_index + 1;
                waypoint_index = robot{i}.controller.waypoint_index;
                
                robot{i}.goal.initial_position =  robot{i}.q.nominal(end,:);
                robot{i}.goal.initial_orientation = robot{i}.pose(1,3);
                robot{i}.goal.final_position = robot{i}.waypoints.positions(waypoint_index,:);
                robot{i}.goal.final_orientation = robot{i}.waypoints.orientations(waypoint_index,:);
                robot{i}.goal.displacement = robot{i}.goal.final_position - robot{i}.goal.initial_position;
                robot{i}.goal.duration = robot{i}.waypoints.times(waypoint_index,1);
                % ---------------------------------------------------------------------------- %
            else
                % ---------------------------------- Message --------------------------------- %
                disp(['Robot ', num2str(i), ' reached the final waypoint. Stopping.']);
                robot{i}.goal.final_waypoint_reached = 1;
                % ---------------------------------------------------------------------------- %
            end
            % ---------------------------------------------------------------------------- %
        end
        % ---------------------------------------------------------------------------- %

    end
    % ---------------------------------------------------------------------------- %
    
    % ------------------------------- ROS Messages ------------------------------- %
    for i=1:num_robots
        vel_msg = rosmessage(robot{i}.velocities_publisher);
        vel_msg.Linear.X = robot{i}.linearVelocity;
        vel_msg.Angular.Z = robot{i}.angularVelocity;
        send(robot{i}.velocities_publisher, vel_msg);

        plot(robot{i}.pose(1), robot{i}.pose(2), 'r*');
        drawnow;
    end
    % ---------------------------------------------------------------------------- %

    % ----------------------------------- Check ---------------------------------- %
    for i=1:num_robots
        if robot{i}.goal.final_waypoint_reached == 0
            break;
        elseif i == num_robots
            robots_plots(robot,num_robots);
            rosshutdown;
        end
    end
    % ---------------------------------------------------------------------------- %

    % ----------------------------------- Wait ----------------------------------- %
    waitfor(loop_rate);
    % ---------------------------------------------------------------------------- %
end
% ---------------------------------------------------------------------------- %


% ---------------------------------------------------------------------------- %
%                                   CALLBACKS                                  %
% ---------------------------------------------------------------------------- %
function gazeboModelStates_SUB_CB(~,msg)
    % ---------------------------------------------------------------------------- %
    %                               Gloabl Variables                               %
    % ---------------------------------------------------------------------------- %
    global gazeboModelStates;
    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %
    %                                  Processing                                  %
    % ---------------------------------------------------------------------------- %
    gazeboModelStates.Data = msg;
    % ---------------------------------------------------------------------------- %
end

function odometry_SUB_CB(~,msg,index)
    % ---------------------------------------------------------------------------- %
    %                               Gloabl Variables                               %
    % ---------------------------------------------------------------------------- %
    global robot;
    % ---------------------------------------------------------------------------- %

    % ---------------------------------------------------------------------------- %
    %                                  Processing                                  %
    % ---------------------------------------------------------------------------- %
    robot{index}.odometry.Data = msg;
    % ---------------------------------------------------------------------------- %
end
% ---------------------------------------------------------------------------- %

% ---------------------------------------------------------------------------- %
%                                   FUNCTIONS                                  %
% ---------------------------------------------------------------------------- %
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
    x = -poseStampedMsg.Pose.Position.X;
    y = -poseStampedMsg.Pose.Position.Y;
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
    odometry_robot_pose_real = [x,y,RPY(1,3)]
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

% ---------------------------------------------------------------------------- %
%                                     PLOTS                                    %
% ---------------------------------------------------------------------------- %
function robots_plots(robot,num_robots)
    for i=1:num_robots
        figure
        subplot(2,1,1)
        hold on
        title(sprintf('Robot%d', i));
        plot(robot{i}.time,robot{i}.q.nominal(:,1),'LineWidth',2);
        plot(robot{i}.time,robot{i}.q.actual(:,1),'LineWidth',2);
        xlabel('[s]');
        ylabel('[m]');
        xlim([0,robot{i}.time(end,1)]);
        legend('nominal','actual')
        grid
    
        subplot(2,1,2)
        plot(robot{i}.time,robot{i}.q.nominal(:,2),'LineWidth',2);
        hold on
        plot(robot{i}.time,robot{i}.q.actual(:,2),'LineWidth',2);
        xlabel('[s]');
        ylabel('[m]');
        xlim([0,robot{i}.time(end,1)]);
        grid
    end
end
% ---------------------------------------------------------------------------- %