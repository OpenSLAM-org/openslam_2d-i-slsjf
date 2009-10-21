%
% function:  select the subset of the beacons from the global map 
% to perform data association -- use the radius of submaps 
%
% date: 2006.03.29 by Shoudong 

function DoSelectBeaconForDataAssociation

global Est;
global Params;


% to record the time used for global state recovery
timeStart = cputime;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% check which submap may have overlap with the current submap
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


Est.OverlapSubmap = []; % to record which submap may have overlap with current one

%% global x y position of robot -- the origin of the current submap
index_final_robot = find(Est.StGlobal(:,1)==-Params.IndexSubmap+1, 1, 'first');

x_r = Est.StGlobal(index_final_robot,5);
y_r = Est.StGlobal(index_final_robot+1,5);


%% distance to (0,0) -- the origion of submap 1
dist_submap_1 = sqrt((x_r)^2+(y_r)^2);

%% check whether the current submap has overlap with submap 1
if dist_submap_1<Est.SubmapRadius(1,2)+Est.SubmapRadius(Params.IndexSubmap,2)+Est.GlobalRobotUncertainty(Params.IndexSubmap-1,2)+Params.EstBias
    Est.OverlapSubmap = [Est.OverlapSubmap;1];
end

%% check whether the current submap has overlap with submap 2,3,...

for i=2:Params.IndexSubmap-1
    % find the origion of submap i -- global x y position of robot in
    % submap i-1

        index_i = find(Est.StGlobal(:,1)==-i+1, 1, 'first');
        x_i = Est.StGlobal(index_i,5);
        y_i = Est.StGlobal(index_i+1,5);
  
   
    % distance from origion of submap i to origion of current submap
    dist_submap_i = sqrt((x_i-x_r)^2+(y_i-y_r)^2);
    % check whether the current submap has overlap with submap i
        sum_radius = Est.SubmapRadius(i,2)+Est.SubmapRadius(Params.IndexSubmap,2)+Est.GlobalRobotUncertainty(i,2)+Est.GlobalRobotUncertainty(Params.IndexSubmap-1,2)+Params.EstBias;
    if dist_submap_i < sum_radius 
        Est.OverlapSubmap = [Est.OverlapSubmap;i];
    end
end
% select the global map beacons within these overlaped submaps
% all the beacons in the overlapped submaps will be recovered
num_overlap_submap = size(Est.OverlapSubmap);
select_beacon = [];
for i=1:num_overlap_submap
    index = find(Est.StGlobal(:,2)==Est.OverlapSubmap(i));
    if size(index,1)>0
        submap_select_beacon = [index, Est.StGlobal(index,1:5)];
        select_beacon = [select_beacon;submap_select_beacon];
    end
end
%% adding the last robot in the selected beacons 
% (for covariance matrix recovery and data association)

select_beacon=[select_beacon;[index_final_robot:index_final_robot+2]',Est.StGlobal(index_final_robot:index_final_robot+2,1:5)];

% Est.SelectedBeaconForDataAssociation = select_beacon;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% further select beacons by computing the distance to robot
%%%%%%% 2007.01.19 Shoudong
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

size_select = size(select_beacon,1);

% %%% for debug: check the number selected beacons
%  if Params.IndexSubmap==2937
%     select_beacon
% %     size_select
%      pause
%  end
% % %%% end for debug

 select_beacon_new=[];
 
 if size_select>3
     for i=1:(size_select-3)/2 % the number of beacons selected        
        x_beac=select_beacon(2*i-1,6);
        y_beac=select_beacon(2*i,6);        
        dist_beac = sqrt((x_beac-x_r)^2+(y_beac-y_r)^2);
        
        if dist_beac < Est.SubmapRadius(Params.IndexSubmap,2)+ Params.EstBias
            select_beacon_new = [select_beacon_new; select_beacon(2*i-1:2*i, :)];
        end
     end
    select_beacon_new = [select_beacon_new; select_beacon(size_select-2:size_select, :)];
 else 
     select_beacon_new = select_beacon;
 end
 
 % %%%% for debug: check the number selected beacons
 if(0)
     if Params.IndexSubmap==2937
         select_beacon_new

         %  size_select_new = size(select_beacon_new,1)
         %
         global_st = Est.StGlobal
         % local_st = Est.St
         pause
     end
 end
 % %%%% end for debug

% %%%% for debug: check the number of beacons selected
 size_select_new = size(select_beacon_new,1);
% %%%% end for debug


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% further select beacons by NN with Euclidean distances
%%%%%%% 2008.01.08 Shoudong
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% if Params.IndexSubmap==2937
%     select_beacon_new
%     pause
% 
%  end

beac = TransMapToObsStateOnly(select_beacon_new(:,6));

%% local map as the observation

localmap_st = Est.St;
num = size(localmap_st,1);

obs = localmap_st(4:num,:);

nmb_obs = size(obs,1)/2; % number of observations
nmb_beac = size(beac,1)/2; % number of beacons

% do the nearest neighbout by Euclidean distance


% find possible_match

possible_match_index=[];

for j = 1:nmb_beac
    r_beac = beac(j*2-1);
    b_beac = beac(j*2);
    if nmb_obs>0
        for i = 1:nmb_obs
            r_obs = obs(i*2-1,4); % or x
            b_obs = obs(i*2,4);  % or y
            diff(i) = norm([r_beac-r_obs;b_beac-b_obs]);
        end
        % find all the possible matched beacons
        if min(diff)< Params.MaxDistance
            possible_match_index = [possible_match_index; 2*j-1; 2*j];
        end
    end

end

select_beacon_new_new = select_beacon_new(possible_match_index,:);

size_select_new = size(select_beacon_new,1);
select_beacon_new_new = [select_beacon_new_new; select_beacon_new(size_select_new-2:size_select_new,:)];

size_select_new_new = size(select_beacon_new_new,1);
%pause

%%%%%%% to prevent select 0 beacons %%%%%%%%%
%%%%%%% need to change the code to include the case of no old beacons
%%%%%%% %%%%%%
%pause
Est.SelectedBeaconForDataAssociation = select_beacon_new_new;

if (size_select_new_new==3)
    disp(' *** no potentially matched beacons');
 %   pause
end

% %%% for check the results
% 
% % Current_Radius = Est.SubmapRadius
% % global_st = Est.StGlobal

%  OverlapSubmap = Est.OverlapSubmap'

% % select_beacon
% % dist_submaps = Est.DistToSubmaps 
% % pause

%% to record the time used in Data association
timeUsed = cputime - timeStart;
Est.timeUsedSelectBeaconForDataAssoc = [Est.timeUsedSelectBeaconForDataAssoc;  timeUsed];

return;

