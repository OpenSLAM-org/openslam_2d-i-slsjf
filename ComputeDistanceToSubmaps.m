%
% function:  to compute the distance from current submap to all the
% previous submaps -- this is used for reordering the global map 
%
% date: 2007.01.03 Shoudong 

function ComputeDistanceToSubmaps

global Est;
global Params;


%% to record the distance from current submap to all the previous submaps
%% this is used for reordering the global map 
Est.DistToSubmaps = [];

% global x y position of robot -- the origin of the current submap
x_r = Est.StGlobal(size(Est.StGlobal,1)-2,5);
y_r = Est.StGlobal(size(Est.StGlobal,1)-1,5);
% distance to (0,0) -- the origion of submap 1
dist_submap_1 = sqrt((x_r)^2+(y_r)^2);
Est.DistToSubmaps = [Est.DistToSubmaps; 1 dist_submap_1];

for i=2:Params.IndexSubmap-1
    % find the origion of submap i -- global x y position of robot in
    % submap i-1
    index_i = find(Est.StGlobal(:,1)==-i+1, 1, 'first');
    x_i = Est.StGlobal(index_i,5);
    y_i = Est.StGlobal(index_i+1,5);
    % distance from origion of submap i to origion of current submap
    dist_submap_i = sqrt((x_i-x_r)^2+(y_i-y_r)^2);
    Est.DistToSubmaps = [Est.DistToSubmaps; i dist_submap_i];
end

return;

