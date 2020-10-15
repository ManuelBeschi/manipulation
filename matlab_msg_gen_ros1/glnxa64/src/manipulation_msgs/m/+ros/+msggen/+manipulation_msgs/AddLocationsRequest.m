
classdef AddLocationsRequest < ros.Message
    %AddLocationsRequest MATLAB implementation of manipulation_msgs/AddLocationsRequest
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'manipulation_msgs/AddLocationsRequest' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '89b26604672dcaffe33782b1b138fff5' % The MD5 Checksum of the message definition
        PropertyList = { 'Locations' } % List of non-constant message properties
        ROSPropertyList = { 'locations' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.manipulation_msgs.Location' ...
			 } % Types of contained nested messages
    end
    properties (Constant)
    end
    properties
        Locations
    end
    methods
        function set.Locations(obj, val)
            if isempty(val)
                % Allow empty [] input
                val = ros.msggen.manipulation_msgs.Location.empty(0, 1);
            end
            val = val(:);
            validAttributes = {'vector'};
            validClasses = {'ros.msggen.manipulation_msgs.Location'};
            validateattributes(val, validClasses, validAttributes, 'AddLocationsRequest', 'Locations')
            obj.Locations = val;
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.manipulation_msgs.AddLocationsRequest.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.manipulation_msgs.AddLocationsRequest;
            obj.reload(strObj);
        end
    end
end
