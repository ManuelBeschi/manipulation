function [data, info] = addBoxRequest
%AddBox gives an empty data for manipulation_msgs/AddBoxRequest
% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
[data.inbound_box_name, info.inbound_box_name] = ros.internal.ros.messages.ros.char('string',0);
[data.box_pose, info.box_pose] = ros.internal.ros.messages.geometry_msgs.pose;
info.box_pose.MLdataType = 'struct';
[data.height, info.height] = ros.internal.ros.messages.ros.default_type('double',1);
info.MessageType = 'manipulation_msgs/AddBoxRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,12);
info.MatPath{1} = 'inbound_box_name';
info.MatPath{2} = 'box_pose';
info.MatPath{3} = 'box_pose.position';
info.MatPath{4} = 'box_pose.position.x';
info.MatPath{5} = 'box_pose.position.y';
info.MatPath{6} = 'box_pose.position.z';
info.MatPath{7} = 'box_pose.orientation';
info.MatPath{8} = 'box_pose.orientation.x';
info.MatPath{9} = 'box_pose.orientation.y';
info.MatPath{10} = 'box_pose.orientation.z';
info.MatPath{11} = 'box_pose.orientation.w';
info.MatPath{12} = 'height';
