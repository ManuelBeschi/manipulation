
classdef PickBox < ros.Message
    %PickBox MATLAB implementation of manipulation_msgs/PickBox
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'manipulation_msgs/PickBox' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '0cae877e70eed9951fbd5fae9fd2ae3d' % The MD5 Checksum of the message definition
        PropertyList = { 'Objects' 'ApproachPose' 'Name' } % List of non-constant message properties
        ROSPropertyList = { 'objects' 'approach_pose' 'name' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.manipulation_msgs.Object' ...
			 'ros.msggen.geometry_msgs.Pose' ...
			 '' ...
			 } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Objects
        ApproachPose
        Name
    end
    methods
        function set.Objects(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.manipulation_msgs.Object.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.manipulation_msgs.Object'};
            validateattributes(val, validClasses, validAttributes, 'PickBox', 'Objects')
            obj.Objects = val;
        end
        function set.ApproachPose(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Pose'};
            validateattributes(val, validClasses, validAttributes, 'PickBox', 'ApproachPose')
            obj.ApproachPose = val;
        end
        function set.Name(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'PickBox', 'Name');
            obj.Name = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.manipulation_msgs.PickBox.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.manipulation_msgs.PickBox;
            obj.reload(strObj);
        end
    end
end
