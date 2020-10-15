
classdef Object < ros.Message
    %Object MATLAB implementation of manipulation_msgs/Object
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'manipulation_msgs/Object' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = 'bf23ef95064f4210d1bed46aedd97d9f' % The MD5 Checksum of the message definition
        PropertyList = { 'GraspingPoses' 'Pose' 'Type' 'Id' } % List of non-constant message properties
        ROSPropertyList = { 'grasping_poses' 'pose' 'type' 'id' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.manipulation_msgs.Grasp' ...
			 'ros.msggen.geometry_msgs.Pose' ...
			 '' ...
			 '' ...
			 } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        GraspingPoses
        Pose
        Type
        Id
    end
    methods
        function set.GraspingPoses(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.manipulation_msgs.Grasp.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.manipulation_msgs.Grasp'};
            validateattributes(val, validClasses, validAttributes, 'Object', 'GraspingPoses')
            obj.GraspingPoses = val;
        end
        function set.Pose(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Pose'};
            validateattributes(val, validClasses, validAttributes, 'Object', 'Pose')
            obj.Pose = val;
        end
        function set.Type(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'Object', 'Type');
            obj.Type = char(val);
        end
        function set.Id(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'Object', 'Id');
            obj.Id = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.manipulation_msgs.Object.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.manipulation_msgs.Object;
            obj.reload(strObj);
        end
    end
end
