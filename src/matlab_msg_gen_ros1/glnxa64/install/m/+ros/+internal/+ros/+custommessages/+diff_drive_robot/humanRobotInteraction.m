function [data, info] = humanRobotInteraction
%HumanRobotInteraction gives an empty data for diff_drive_robot/HumanRobotInteraction
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'diff_drive_robot/HumanRobotInteraction';
[data.HumanID, info.HumanID] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.RobotVelocity, info.RobotVelocity] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.WaitingTime, info.WaitingTime] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.StartFilling, info.StartFilling] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.FinishFilling, info.FinishFilling] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.StartServing, info.StartServing] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.FinishServing, info.FinishServing] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.TimeFilling, info.TimeFilling] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.TimeServing, info.TimeServing] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.Confirm, info.Confirm] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Task, info.Task] = ros.internal.ros.messages.ros.default_type('int32',1);
info.MessageType = 'diff_drive_robot/HumanRobotInteraction';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,11);
info.MatPath{1} = 'HumanID';
info.MatPath{2} = 'RobotVelocity';
info.MatPath{3} = 'WaitingTime';
info.MatPath{4} = 'StartFilling';
info.MatPath{5} = 'FinishFilling';
info.MatPath{6} = 'StartServing';
info.MatPath{7} = 'FinishServing';
info.MatPath{8} = 'TimeFilling';
info.MatPath{9} = 'TimeServing';
info.MatPath{10} = 'Confirm';
info.MatPath{11} = 'Task';
