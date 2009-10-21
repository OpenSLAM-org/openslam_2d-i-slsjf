% function: DoGlobalMapObservation
% new beacon/robot initialization, update, state recovery
% 
% date: 2006.03.28 Shoudong --- modified from (2005.9.14, zhan)

function DoGlobalMapObservation

global Est; 

global Params;
global Match;

% disp(' *** entering DoGlobalMapObservation');

if(isempty(Est.StGlobal))
    disp('In DoGlobalMapObservation, no data in Est.StGlobal');
    return;
end

% check which is new beacon which is old beacon

num_row_total = size(Est.StGlobal,1);

% the number of beacons

    num_beacon_total = (num_row_total-3*(Params.IndexSubmap-1))/2; % many poses in the state

match_local_map = Match.LocalMap;

newbeacon = []; % this is measurement about new beacons
oldbeacon = []; 

% assign the index in global map for the new beacons

for i = 1:size(match_local_map,1)-1  % robot is in the last row
    if match_local_map(i,3)==-100
        num_beacon_total = num_beacon_total + 1;
        newbeacon = [newbeacon; num_beacon_total, match_local_map(i,1:2), match_local_map(i,4), Est.St(2*i+2,4), Est.St(2*i+3,4)];
% format of newbeacon is: [beacon index, index, x_hat, y_hat]. x_hat and y_hat is the coordinate in the robot coordinate system
        match_local_map(i,3) = num_beacon_total;
    end    
end

% update the match matrix
Match.LocalMap = match_local_map;

%% if there are some new beacons, use the prvious robot pose (the last one in the current global map)
% to initialize them

% global x y position of robot -- the origin of the current submap
index_final_robot = find(Est.StGlobal(:,1)==-Params.IndexSubmap+1, 1, 'first');

x_r = Est.StGlobal(index_final_robot,5);
y_r = Est.StGlobal(index_final_robot+1,5);
phi_r = Est.StGlobal(index_final_robot+2,5);

robot_start = [x_r, y_r, phi_r];

if(~isempty(newbeacon))        
    for i=1:(size(newbeacon,1))        
        DoGlobalMapInitialization(newbeacon(i,:),robot_start);
    end
end

newrobot = [-Params.IndexSubmap, Est.St(1,4), Est.St(2,4), Est.St(3,4)];

% initialize the new robot location
DoGlobalMapRobotInitialization(newrobot,robot_start);

%% update all the beacons and robots using the last submap
DoGlobalMapUpdate;   

return;




