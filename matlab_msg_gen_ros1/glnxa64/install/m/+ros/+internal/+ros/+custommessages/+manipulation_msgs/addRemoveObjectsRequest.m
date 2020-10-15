function [data, info] = addRemoveObjectsRequest
%AddRemoveObjects gives an empty data for manipulation_msgs/AddRemoveObjectsRequest
% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
[data.inbound_box_name, info.inbound_box_name] = ros.internal.ros.messages.ros.char('string',0);
[data.add_objects, info.add_objects] = ros.internal.ros.custommessages.manipulation_msgs.object;
info.add_objects.MLdataType = 'struct';
info.add_objects.MaxLen = NaN;
info.add_objects.MinLen = 0;
data.add_objects = data.add_objects([],1);
[data.remove_objects, info.remove_objects] = ros.internal.ros.custommessages.manipulation_msgs.object;
info.remove_objects.MLdataType = 'struct';
info.remove_objects.MaxLen = NaN;
info.remove_objects.MinLen = 0;
data.remove_objects = data.remove_objects([],1);
info.MessageType = 'manipulation_msgs/AddRemoveObjectsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,51);
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
info.MatPath{27} = 'remove_objects';
info.MatPath{28} = 'remove_objects.grasping_poses';
info.MatPath{29} = 'remove_objects.grasping_poses.pose';
info.MatPath{30} = 'remove_objects.grasping_poses.pose.position';
info.MatPath{31} = 'remove_objects.grasping_poses.pose.position.x';
info.MatPath{32} = 'remove_objects.grasping_poses.pose.position.y';
info.MatPath{33} = 'remove_objects.grasping_poses.pose.position.z';
info.MatPath{34} = 'remove_objects.grasping_poses.pose.orientation';
info.MatPath{35} = 'remove_objects.grasping_poses.pose.orientation.x';
info.MatPath{36} = 'remove_objects.grasping_poses.pose.orientation.y';
info.MatPath{37} = 'remove_objects.grasping_poses.pose.orientation.z';
info.MatPath{38} = 'remove_objects.grasping_poses.pose.orientation.w';
info.MatPath{39} = 'remove_objects.grasping_poses.tool_name';
info.MatPath{40} = 'remove_objects.type';
info.MatPath{41} = 'remove_objects.id';
info.MatPath{42} = 'remove_objects.pose';
info.MatPath{43} = 'remove_objects.pose.position';
info.MatPath{44} = 'remove_objects.pose.position.x';
info.MatPath{45} = 'remove_objects.pose.position.y';
info.MatPath{46} = 'remove_objects.pose.position.z';
info.MatPath{47} = 'remove_objects.pose.orientation';
info.MatPath{48} = 'remove_objects.pose.orientation.x';
info.MatPath{49} = 'remove_objects.pose.orientation.y';
info.MatPath{50} = 'remove_objects.pose.orientation.z';
info.MatPath{51} = 'remove_objects.pose.orientation.w';