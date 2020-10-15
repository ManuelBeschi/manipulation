function [data, info] = removeLocationsRequest
%RemoveLocations gives an empty data for manipulation_msgs/RemoveLocationsRequest
% Copyright 2019-2020 The MathWorks, Inc.
data = struct();
[data.location_names, info.location_names] = ros.internal.ros.messages.ros.char('string',NaN);
info.MessageType = 'manipulation_msgs/RemoveLocationsRequest';
info.constant = 0;
info.default = 0;
info.maxstrlen = NaN;
info.MaxLen = 1;
info.MinLen = 1;
info.MatPath = cell(1,1);
info.MatPath{1} = 'location_names';
