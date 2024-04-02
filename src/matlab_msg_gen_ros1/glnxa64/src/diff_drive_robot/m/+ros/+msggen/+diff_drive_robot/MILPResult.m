
classdef MILPResult < ros.Message
    %MILPResult MATLAB implementation of diff_drive_robot/MILPResult
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'diff_drive_robot/MILPResult' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '2d08fbd0bb0c654ece09998565b41c04' % The MD5 Checksum of the message definition
        PropertyList = { 'RobotID' 'Humans' 'GoingStart' 'WaitingStart' 'ServingStart' 'DepotStart' 'GoingFinish' 'WaitingFinish' 'ServingFinish' 'DepotFinish' } % List of non-constant message properties
        ROSPropertyList = { 'RobotID' 'Humans' 'GoingStart' 'WaitingStart' 'ServingStart' 'DepotStart' 'GoingFinish' 'WaitingFinish' 'ServingFinish' 'DepotFinish' } % List of non-constant ROS message properties
        PropertyMessageTypes = { '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            '' ...
            } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        RobotID
        Humans
        GoingStart
        WaitingStart
        ServingStart
        DepotStart
        GoingFinish
        WaitingFinish
        ServingFinish
        DepotFinish
    end
    methods
        function set.RobotID(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'MILPResult', 'RobotID');
            obj.RobotID = int32(val);
        end
        function set.Humans(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = int32.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'MILPResult', 'Humans');
            obj.Humans = int32(val);
        end
        function set.GoingStart(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'MILPResult', 'GoingStart');
            obj.GoingStart = double(val);
        end
        function set.WaitingStart(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'MILPResult', 'WaitingStart');
            obj.WaitingStart = double(val);
        end
        function set.ServingStart(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'MILPResult', 'ServingStart');
            obj.ServingStart = double(val);
        end
        function set.DepotStart(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'MILPResult', 'DepotStart');
            obj.DepotStart = double(val);
        end
        function set.GoingFinish(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'MILPResult', 'GoingFinish');
            obj.GoingFinish = double(val);
        end
        function set.WaitingFinish(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'MILPResult', 'WaitingFinish');
            obj.WaitingFinish = double(val);
        end
        function set.ServingFinish(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'MILPResult', 'ServingFinish');
            obj.ServingFinish = double(val);
        end
        function set.DepotFinish(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'MILPResult', 'DepotFinish');
            obj.DepotFinish = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.diff_drive_robot.MILPResult.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.diff_drive_robot.MILPResult(strObj);
        end
    end
end
