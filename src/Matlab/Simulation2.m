clc;
rosshutdown;
clear;
close all;

% Initialize ROS
if ros.internal.Global.isNodeActive == 0
    rosinit('http://192.168.0.137:11311');
end

% Variables Declaration
frequency = 5;
pause(0.3)
r = rosrate(frequency); % Create a rate object that runs at 10 Hz
Ts = 1/frequency;
loop_rate = rateControl(frequency);
num_robots = 1; % Number of robots
Sim = false;

% Define different timing for each robot
timing = [40]; % Desired times to reach the final positions in seconds for each robot

% Define robot's starting and desired positions for L-shaped trajectory
depot_positions = [-1, 0.5]; % x, y for each robot
desired_positions = [1, 1.5]; % x, y for each robot

% Initialize arrays for dynamic handling
modelStatesSubSim = cell(num_robots, 1);
modelStatesSubReal = cell(num_robots, 1);
robotIndex = zeros(num_robots, 1);
vel_pub = cell(num_robots, 1);
robotName = cell(num_robots, 1);
controller = cell(num_robots, 1);
waypoints = cell(num_robots, 1);
finalWaypoint = zeros(num_robots, 2);
startTime = zeros(num_robots, 1, 'uint64'); % Start time for each robot
robotStopped = zeros(num_robots, 1); % Track if each robot has stopped

% Subscribe to Gazebo model states topic once for efficiency
modelStatesSubSim{1} = rossubscriber('/gazebo/model_states', 'gazebo_msgs/ModelStates');

modelStatesSubReal{1} = rossubscriber('/vrpn_client_node/Turtlebot1/pose', 'geometry_msgs/PoseStamped');

% Setup for each robot
for i = 1:num_robots
    robotName{i} = sprintf('diff_drive_robot%d', i);
    vel_pub{i} = rospublisher(sprintf("/nuc%d/cmd_vel_mux/input/teleop", i), "geometry_msgs/Twist")
    if Sim == true
        modelStatesData = receive(modelStatesSubSim{1}, 5);
        robotIndex(i) = find(strcmp(modelStatesData.Name, robotName{i}));
    else
        modelStatesData = receive(modelStatesSubReal{1}, 5);
    end

    % Initialize Pure Pursuit controller
    controller{i} = controllerPurePursuit;
    controller{i}.MaxAngularVelocity = 0.65;
    controller{i}.LookaheadDistance = 0.2;
    
    startPosition = depot_positions(i,:);
    endPosition = desired_positions(i,:);
    intermediatePosition = [endPosition(1), startPosition(2)]; % Create L-shaped path
    waypoints{i} = [startPosition; intermediatePosition; endPosition];
    controller{i}.Waypoints = waypoints{i};
    finalWaypoint(i,:) = endPosition;
    
    startTime(i) = tic; % Record start time for each robot
    
    %figure;
    plot(waypoints{i}(:,1), waypoints{i}(:,2), '-o', 'LineWidth', 2);
    xlim([-1, 3]);
    ylim([-1, 3]);
    xlabel('X (m)');
    ylabel('Y (m)');
    title(sprintf('Planned L-shaped Trajectory for Robot %d', i));
    grid on;
    hold on;
end

distanceThreshold = 0.1; % Define a threshold for when the waypoint is considered reached

reset(r);
startTime = r.TotalElapsedTime; % Record the start time for the control loop
% Main control loop
while ros.internal.Global.isNodeActive && any(~robotStopped)
    for i = 1:num_robots
        if robotStopped(i) % Skip if this robot has already stopped
            continue;
        end
        if Sim == true
            modelStatesData = receive(modelStatesSubSim{1}, 0.05);
            robotPose = position_robot_simulation(modelStatesData, robotIndex(i));
        else
            modelStatesData = receive(modelStatesSubReal{1}, 1.5);
            robotPose = position_robot_real(modelStatesData);
        end

        currentPose = [robotPose(1), robotPose(2), deg2rad(robotPose(3))]

        if isequal(currentPose, [NaN, NaN, NaN]) % Check if current pose is valid
            continue; % Skip this iteration if the robot's position isn't valid
        end
        
        [v, omega] = controller{i}(currentPose);

        elapsed = r.TotalElapsedTime - startTime
        remainingTime = max(timing(i) - elapsed, 0.1); % Use individual timing for each robot
        distanceToFinalWaypoint = sqrt((finalWaypoint(i,1) - robotPose(1))^2 + (finalWaypoint(i,2) - robotPose(2))^2);
        
        if distanceToFinalWaypoint < distanceThreshold
            v = 0; omega = 0; % Stop robot by setting velocities to zero
            robotStopped(i) = 1; % Mark robot as stopped
            disp(['Robot ', num2str(i), ' reached the final waypoint. Stopping.']);
        else
            % Calculate and set desired velocity to reach the destination on time
            desiredVelocity = distanceToFinalWaypoint / (remainingTime + 5);
            controller{i}.DesiredLinearVelocity = min(desiredVelocity, 0.5); % Limit max velocity
        end
        
        vel_msg = rosmessage(vel_pub{i});
        vel_msg.Linear.X = v;
        vel_msg.Angular.Z = omega;
        send(vel_pub{i}, vel_msg);

        plot(robotPose(1), robotPose(2), 'r*');
        drawnow;
    end
    waitfor(r);
end

% Function to extract robot's position and orientation
function position_robot_simulation = position_robot_simulation(modelStatesData, robotIndex)
    if ~isempty(robotIndex)
        x = modelStatesData.Pose(robotIndex).Position.X;
        y = modelStatesData.Pose(robotIndex).Position.Y;
        qx = modelStatesData.Pose(robotIndex).Orientation.X;
        qy = modelStatesData.Pose(robotIndex).Orientation.Y;
        qz = modelStatesData.Pose(robotIndex).Orientation.Z;
        qw = modelStatesData.Pose(robotIndex).Orientation.W;
        yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy^2 + qz^2));
        yaw_deg = rad2deg(yaw);
        position_robot_simulation = [x, y, yaw_deg];
    else
        position_robot_simulation = [NaN, NaN, NaN]; % Return NaN if robot index is not found
    end
end

function position_robot_real = position_robot_real(modelStatesData)
    x = -modelStatesData.Pose.Position.X;
    y = -modelStatesData.Pose.Position.Y;
    qx = modelStatesData.Pose.Orientation.X;
    qy = modelStatesData.Pose.Orientation.Y;
    qz = modelStatesData.Pose.Orientation.Z;
    qw = modelStatesData.Pose.Orientation.W;
    yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy^2 + qz^2));
    yaw_deg = rad2deg(yaw);
    position_robot_real = [x, y, yaw_deg];  
end
