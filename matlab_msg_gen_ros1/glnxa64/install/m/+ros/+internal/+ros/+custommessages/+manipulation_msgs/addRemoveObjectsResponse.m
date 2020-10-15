function [data, info] = addRemoveObjectsResponse
%AddRemoveObjects gives an empty data for manipulation_msgs/AddRemoveObjectsResponse
% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
[data.results, info.results] = ros.internal.ros.messages.ros.default_type('int8',1);
[data.Success, info.Success] = ros.internal.ros.messages.ros.default_type('int8',1, 0);
[data.ObjectNotFound, info.ObjectNotFound] = ros.internal.ros.messages.ros.default_type('int8',1, -1);
info.MessageType = 'manipulation_msgs/AddRemoveObjectsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'results';
info.MatPath{2} = 'Success';
info.MatPath{3} = 'ObjectNotFound';
