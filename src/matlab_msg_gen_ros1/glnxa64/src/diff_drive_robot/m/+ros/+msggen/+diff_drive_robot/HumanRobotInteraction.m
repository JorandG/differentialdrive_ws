
classdef HumanRobotInteraction < ros.Message
    %HumanRobotInteraction MATLAB implementation of diff_drive_robot/HumanRobotInteraction
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'diff_drive_robot/HumanRobotInteraction' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'd476374a890a24a3723780d1c56bf0f8' % The MD5 Checksum of the message definition
        PropertyList = { 'HumanID' 'Robots' 'RobotWaitingDistance' 'RobotVelocityProximity' 'RobotMinVelocityProximity' 'RobotMaxVelocityProximity' 'RobotVelocityProximityWeight' 'WaitingTime' 'WaitingTimeWeight' 'StartFilling' 'FinishFilling' 'StartServing' 'FinishServing' 'TimeFilling' 'TimeServing' 'ConfirmServing' 'ConfirmFilling' 'Task' 'TaskFilling' 'Happiness' 'Efficiency' 'Severity' } % List of non-constant message properties
        ROSPropertyList = { 'HumanID' 'Robots' 'RobotWaitingDistance' 'RobotVelocityProximity' 'RobotMinVelocityProximity' 'RobotMaxVelocityProximity' 'RobotVelocityProximityWeight' 'WaitingTime' 'WaitingTimeWeight' 'StartFilling' 'FinishFilling' 'StartServing' 'FinishServing' 'TimeFilling' 'TimeServing' 'ConfirmServing' 'ConfirmFilling' 'Task' 'TaskFilling' 'Happiness' 'Efficiency' 'Severity' } % List of non-constant ROS message properties
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
            '' ...
            '' ...
            '' ...
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
        HumanID
        Robots
        RobotWaitingDistance
        RobotVelocityProximity
        RobotMinVelocityProximity
        RobotMaxVelocityProximity
        RobotVelocityProximityWeight
        WaitingTime
        WaitingTimeWeight
        StartFilling
        FinishFilling
        StartServing
        FinishServing
        TimeFilling
        TimeServing
        ConfirmServing
        ConfirmFilling
        Task
        TaskFilling
        Happiness
        Efficiency
        Severity
    end
    methods
        function set.HumanID(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'HumanRobotInteraction', 'HumanID');
            obj.HumanID = int32(val);
        end
        function set.Robots(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'HumanRobotInteraction', 'Robots');
            obj.Robots = double(val);
        end
        function set.RobotWaitingDistance(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'HumanRobotInteraction', 'RobotWaitingDistance');
            obj.RobotWaitingDistance = double(val);
        end
        function set.RobotVelocityProximity(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'HumanRobotInteraction', 'RobotVelocityProximity');
            obj.RobotVelocityProximity = double(val);
        end
        function set.RobotMinVelocityProximity(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'HumanRobotInteraction', 'RobotMinVelocityProximity');
            obj.RobotMinVelocityProximity = double(val);
        end
        function set.RobotMaxVelocityProximity(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'HumanRobotInteraction', 'RobotMaxVelocityProximity');
            obj.RobotMaxVelocityProximity = double(val);
        end
        function set.RobotVelocityProximityWeight(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'HumanRobotInteraction', 'RobotVelocityProximityWeight');
            obj.RobotVelocityProximityWeight = double(val);
        end
        function set.WaitingTime(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'HumanRobotInteraction', 'WaitingTime');
            obj.WaitingTime = double(val);
        end
        function set.WaitingTimeWeight(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'HumanRobotInteraction', 'WaitingTimeWeight');
            obj.WaitingTimeWeight = double(val);
        end
        function set.StartFilling(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'HumanRobotInteraction', 'StartFilling');
            obj.StartFilling = double(val);
        end
        function set.FinishFilling(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'HumanRobotInteraction', 'FinishFilling');
            obj.FinishFilling = double(val);
        end
        function set.StartServing(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'HumanRobotInteraction', 'StartServing');
            obj.StartServing = double(val);
        end
        function set.FinishServing(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'HumanRobotInteraction', 'FinishServing');
            obj.FinishServing = double(val);
        end
        function set.TimeFilling(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'HumanRobotInteraction', 'TimeFilling');
            obj.TimeFilling = double(val);
        end
        function set.TimeServing(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'HumanRobotInteraction', 'TimeServing');
            obj.TimeServing = double(val);
        end
        function set.ConfirmServing(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = int32.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'HumanRobotInteraction', 'ConfirmServing');
            obj.ConfirmServing = int32(val);
        end
        function set.ConfirmFilling(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = int32.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'HumanRobotInteraction', 'ConfirmFilling');
            obj.ConfirmFilling = int32(val);
        end
        function set.Task(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'HumanRobotInteraction', 'Task');
            obj.Task = int32(val);
        end
        function set.TaskFilling(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'HumanRobotInteraction', 'TaskFilling');
            obj.TaskFilling = int32(val);
        end
        function set.Happiness(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'HumanRobotInteraction', 'Happiness');
            obj.Happiness = double(val);
        end
        function set.Efficiency(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'HumanRobotInteraction', 'Efficiency');
            obj.Efficiency = double(val);
        end
        function set.Severity(obj, val)
            validClasses = {'numeric'};
            if isempty(val)
                % Allow empty [] input
                val = double.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validateattributes(val, validClasses, validAttributes, 'HumanRobotInteraction', 'Severity');
            obj.Severity = double(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.diff_drive_robot.HumanRobotInteraction.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.diff_drive_robot.HumanRobotInteraction(strObj);
        end
    end
end
