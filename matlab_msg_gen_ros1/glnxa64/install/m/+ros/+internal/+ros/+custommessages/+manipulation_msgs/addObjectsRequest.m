function [data, info] = addObjectsRequest
%AddObjects gives an empty data for manipulation_msgs/AddObjectsRequest
% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
[data.inbound_box_name, info.inbound_box_name] = ros.internal.ros.messages.ros.char('string',0);
[data.add_objects, info.add_objects] = ros.internal.ros.custommessages.manipulation_msgs.object;
info.add_objects.MLdataType = 'struct';
info.add_objects.MaxLen = NaN;
info.add_objects.MinLen = 0;
data.add_objects = data.add_objects([],1);
info.MessageType = 'manipulation_msgs/AddObjectsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,26);
info.MatPath{1} = 'inbound_box_name';
info.MatPath{2} = 'add_objects';
info.MatPath{3} = 'add_objects.grasping_poses';
info.MatPath{4} = 'add_objects.grasping_poses.pose';
info.MatPath{5} = 'add_objects.grasping_poses.pose.position';
info.MatPath{6} = 'add_objects.grasping_poses.pose.position.x';
info.MatPath{7} = 'add_objects.grasping_poses.pose.position.y';
info.MatPath{8} = 'add_objects.grasping_poses.pose.position.z';
info.MatPath{9} = 'add_objects.grasping_poses.pose.orientation';
info.MatPath{10} = 'add_objects.grasping_poses.pose.orientation.x';
info.MatPath{11} = 'add_objects.grasping_poses.pose.orientation.y';
info.MatPath{12} = 'add_objects.grasping_poses.pose.orientation.z';
info.MatPath{13} = 'add_objects.grasping_poses.pose.orientation.w';
info.MatPath{14} = 'add_objects.grasping_poses.tool_name';
info.MatPath{15} = 'add_objects.type';
info.MatPath{16} = 'add_objects.id';
info.MatPath{17} = 'add_objects.pose';
info.MatPath{18} = 'add_objects.pose.position';
info.MatPath{19} = 'add_objects.pose.position.x';
info.MatPath{20} = 'add_objects.pose.position.y';
info.MatPath{21} = 'add_objects.pose.position.z';
info.MatPath{22} = 'add_objects.pose.orientation';
info.MatPath{23} = 'add_objects.pose.orientation.x';
info.MatPath{24} = 'add_objects.pose.orientation.y';
info.MatPath{25} = 'add_objects.pose.orientation.z';
info.MatPath{26} = 'add_objects.pose.orientation.w';
