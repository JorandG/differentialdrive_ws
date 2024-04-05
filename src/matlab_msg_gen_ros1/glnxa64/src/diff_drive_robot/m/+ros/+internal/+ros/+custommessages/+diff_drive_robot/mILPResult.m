function [data, info] = mILPResult
%MILPResult gives an empty data for diff_drive_robot/MILPResult
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'diff_drive_robot/MILPResult';
[data.RobotID, info.RobotID] = ros.internal.ros.messages.ros.default_type('int32',1);
[data.Humans, info.Humans] = ros.internal.ros.messages.ros.default_type('int32',NaN);
[data.GoingStart, info.GoingStart] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.WaitingStart, info.WaitingStart] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.ServingStart, info.ServingStart] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.DepotStart, info.DepotStart] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.GoingFinish, info.GoingFinish] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.WaitingFinish, info.WaitingFinish] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.ServingFinish, info.ServingFinish] = ros.internal.ros.messages.ros.default_type('double',NaN);
[data.DepotFinish, info.DepotFinish] = ros.internal.ros.messages.ros.default_type('double',NaN);
info.MessageType = 'diff_drive_robot/MILPResult';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,10);
info.MatPath{1} = 'RobotID';
info.MatPath{2} = 'Humans';
info.MatPath{3} = 'GoingStart';
info.MatPath{4} = 'WaitingStart';
info.MatPath{5} = 'ServingStart';
info.MatPath{6} = 'DepotStart';
info.MatPath{7} = 'GoingFinish';
info.MatPath{8} = 'WaitingFinish';
info.MatPath{9} = 'ServingFinish';
info.MatPath{10} = 'DepotFinish';