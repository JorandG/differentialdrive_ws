function [data, info] = timeMessage
%TimeMessage gives an empty data for diff_drive_robot/TimeMessage
% Copyright 2019-2020 The MathWorks, Inc.
%#codegen
data = struct();
data.MessageType = 'diff_drive_robot/TimeMessage';
[data.Time, info.Time] = ros.internal.ros.messages.ros.char('string',0);
info.MessageType = 'diff_drive_robot/TimeMessage';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'time';
