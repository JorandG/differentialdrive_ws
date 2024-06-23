function [data, info] = humanRobotInteraction
%HumanRobotInteraction gives an empty data for diff_drive_robot/HumanRobotInteraction
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'diff_drive_robot/HumanRobotInteraction';
[data.HumanID, info.HumanID] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Robots, info.Robots] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.RobotWaitingDistance, info.RobotWaitingDistance] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.RobotVelocityProximity, info.RobotVelocityProximity] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.RobotMinVelocityProximity, info.RobotMinVelocityProximity] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.RobotMaxVelocityProximity, info.RobotMaxVelocityProximity] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.RobotVelocityProximityWeight, info.RobotVelocityProximityWeight] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.WaitingTime, info.WaitingTime] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.WaitingTimeWeight, info.WaitingTimeWeight] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.StartFilling, info.StartFilling] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.FinishFilling, info.FinishFilling] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.StartServing, info.StartServing] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.FinishServing, info.FinishServing] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.TimeFilling, info.TimeFilling] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.TimeServing, info.TimeServing] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.ConfirmServing, info.ConfirmServing] = ros.internal.ros.messages.ros.default_type('int32',NaN);
[data.ConfirmFilling, info.ConfirmFilling] = ros.internal.ros.messages.ros.default_type('int32',NaN);
[data.Task, info.Task] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.TaskFilling, info.TaskFilling] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Happiness, info.Happiness] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Efficiency, info.Efficiency] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Severity, info.Severity] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'diff_drive_robot/HumanRobotInteraction';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,22);
info.MatPath{1} = 'HumanID';
info.MatPath{2} = 'Robots';
info.MatPath{3} = 'RobotWaitingDistance';
info.MatPath{4} = 'RobotVelocityProximity';
info.MatPath{5} = 'RobotMinVelocityProximity';
info.MatPath{6} = 'RobotMaxVelocityProximity';
info.MatPath{7} = 'RobotVelocityProximityWeight';
info.MatPath{8} = 'WaitingTime';
info.MatPath{9} = 'WaitingTimeWeight';
info.MatPath{10} = 'StartFilling';
info.MatPath{11} = 'FinishFilling';
info.MatPath{12} = 'StartServing';
info.MatPath{13} = 'FinishServing';
info.MatPath{14} = 'TimeFilling';
info.MatPath{15} = 'TimeServing';
info.MatPath{16} = 'ConfirmServing';
info.MatPath{17} = 'ConfirmFilling';
info.MatPath{18} = 'Task';
info.MatPath{19} = 'TaskFilling';
info.MatPath{20} = 'Happiness';
info.MatPath{21} = 'Efficiency';
info.MatPath{22} = 'Severity';
