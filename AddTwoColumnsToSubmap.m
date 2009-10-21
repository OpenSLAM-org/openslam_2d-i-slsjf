%
% function: add two columns to the local map
%
% input: the local map
%
% output: the new local map
%
% original state [ID, estimate]
% new local map state is
% [local_map_number, beacon_index_local, ID, estimate]
%
% date: 2006.03.18 Shoudong



function [new_localmap_st]=AddTwoColumnsToSubmap(localmap_st)

global Params;


%% add the local_map_number (e.g. submap 1)

size_local = size(localmap_st,1);
new_localmap_st = [Params.IndexSubmap*ones(size_local,1),zeros(size_local,1),localmap_st];

%% add the beacon index in local map

for i=4:2:size_local
    new_localmap_st(i:i+1,2) = [(i-2)/2;(i-2)/2];
end

return;
