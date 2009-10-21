%
% function: data association
% assume data association -- use the true index to find the data association
%
% input:
% globalmap, submap
%
% output: match_matrix
%
% date: 2007.03.17 Shoudong, 

function DoGlobalDataAssociationAssumed

global Params;
global Match;
global Est;
    
%disp(' *** entering DoGlobalDataAssociationAssumed');


%%%%% assume data association 
%%%%% use the true index to find the data association

%% local map as the observation

localmap_st = Est.St;
n2 = size(localmap_st,1);
obs = localmap_st(4:n2,4);


%% find the global index and the feature ID 

oldbeacon=[];

for i = 4:2:size(Est.St,1)  % robot is in it, so start from 4 
     oldbeacon_sign = 0;
    for j = 1:size(Est.StGlobal,1)
        if(Est.St(i,3)==Est.StGlobal(j,4))
            % old beacon!
            % format of oldbeacon --- [local index, global index, ID]
            oldbeacon_sign = 1;
            oldbeacon = [oldbeacon; (i-2)/2 Est.StGlobal(j,1) Est.St(i,3)];
            break;
        end
    end
    if(oldbeacon_sign == 0)
        oldbeacon = [oldbeacon; (i-2)/2 -100 Est.St(i,3)];
    end    
end

% oldbeacon

%pause

%% assume data association -- 2007.01.03
match_local_map = [Params.IndexSubmap*ones(size(oldbeacon,1),1), oldbeacon]; % assume data association


match_local_map = [match_local_map; Params.IndexSubmap, 0, -Params.IndexSubmap, -Params.IndexSubmap];
%pause

Match.LocalMap = match_local_map;

% format: [submap number, index_in_local_map, index_global_map, ID] 

%pause
return;

