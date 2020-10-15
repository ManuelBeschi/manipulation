
classdef AddRemoveObjectsRequest < ros.Message
    %AddRemoveObjectsRequest MATLAB implementation of pickplace_msgs/AddRemoveObjectsRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'pickplace_msgs/AddRemoveObjectsRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '51aa5ad13aecf69d39ca9616fdb01a34' % The MD5 Checksum of the message definition
        PropertyList = { 'AddObjects' 'RemoveObjects' 'InboundBoxName' } % List of non-constant message properties
        ROSPropertyList = { 'add_objects' 'remove_objects' 'inbound_box_name' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.pickplace_msgs.Object' ...
			 'ros.msggen.pickplace_msgs.Object' ...
			 '' ...
			 } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        AddObjects
        RemoveObjects
        InboundBoxName
    end
    methods
        function set.AddObjects(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.pickplace_msgs.Object.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.pickplace_msgs.Object'};
            validateattributes(val, validClasses, validAttributes, 'AddRemoveObjectsRequest', 'AddObjects')
            obj.AddObjects = val;
        end
        function set.RemoveObjects(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.pickplace_msgs.Object.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.pickplace_msgs.Object'};
            validateattributes(val, validClasses, validAttributes, 'AddRemoveObjectsRequest', 'RemoveObjects')
            obj.RemoveObjects = val;
        end
        function set.InboundBoxName(obj, val)
            val = convertStringsToChars(val);
            validClasses = {'char', 'string'};
            validAttributes = {};
            validateattributes(val, validClasses, validAttributes, 'AddRemoveObjectsRequest', 'InboundBoxName');
            obj.InboundBoxName = char(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.pickplace_msgs.AddRemoveObjectsRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.pickplace_msgs.AddRemoveObjectsRequest;
            obj.reload(strObj);
        end
    end
end