function [data, info] = listOfObjectsResponse
%ListOfObjects gives an empty data for pickplace_msgs/ListOfObjectsResponse
% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
[data.inbound_boxes, info.inbound_boxes] = ros.internal.ros.custommessages.pickplace_msgs.pickBox;
info.inbound_boxes.MLdataType = 'struct';
info.inbound_boxes.MaxLen = NaN;
info.inbound_boxes.MinLen = 0;
data.inbound_boxes = data.inbound_boxes([],1);
info.MessageType = 'pickplace_msgs/ListOfObjectsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,27);
info.MatPath{1} = 'inbound_boxes';
info.MatPath{2} = 'inbound_boxes.objects';
info.MatPath{3} = 'inbound_boxes.objects.grasping_poses';
info.MatPath{4} = 'inbound_boxes.objects.grasping_poses.pose';
info.MatPath{5} = 'inbound_boxes.objects.grasping_poses.pose.position';
info.MatPath{6} = 'inbound_boxes.objects.grasping_poses.pose.position.x';
info.MatPath{7} = 'inbound_boxes.objects.grasping_poses.pose.position.y';
info.MatPath{8} = 'inbound_boxes.objects.grasping_poses.pose.position.z';
info.MatPath{9} = 'inbound_boxes.objects.grasping_poses.pose.orientation';
info.MatPath{10} = 'inbound_boxes.objects.grasping_poses.pose.orientation.x';
info.MatPath{11} = 'inbound_boxes.objects.grasping_poses.pose.orientation.y';
info.MatPath{12} = 'inbound_boxes.objects.grasping_poses.pose.orientation.z';
info.MatPath{13} = 'inbound_boxes.objects.grasping_poses.pose.orientation.w';
info.MatPath{14} = 'inbound_boxes.objects.grasping_poses.tool_name';
info.MatPath{15} = 'inbound_boxes.objects.type';
info.MatPath{16} = 'inbound_boxes.objects.id';
info.MatPath{17} = 'inbound_boxes.name';
info.MatPath{18} = 'inbound_boxes.approach_pose';
info.MatPath{19} = 'inbound_boxes.approach_pose.position';
info.MatPath{20} = 'inbound_boxes.approach_pose.position.x';
info.MatPath{21} = 'inbound_boxes.approach_pose.position.y';
info.MatPath{22} = 'inbound_boxes.approach_pose.position.z';
info.MatPath{23} = 'inbound_boxes.approach_pose.orientation';
info.MatPath{24} = 'inbound_boxes.approach_pose.orientation.x';
info.MatPath{25} = 'inbound_boxes.approach_pose.orientation.y';
info.MatPath{26} = 'inbound_boxes.approach_pose.orientation.z';
info.MatPath{27} = 'inbound_boxes.approach_pose.orientation.w';