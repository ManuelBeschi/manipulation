function [data, info] = listOfObjectsResponse
%ListOfObjects gives an empty data for manipulation_msgs/ListOfObjectsResponse
% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
[data.type, info.type] = ros.internal.ros.messages.ros.char('string',NaN);
[data.id, info.id] = ros.internal.ros.messages.ros.char('string',NaN);
[data.box, info.box] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'manipulation_msgs/ListOfObjectsResponse';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,3);
info.MatPath{1} = 'type';
info.MatPath{2} = 'id';
info.MatPath{3} = 'box';
