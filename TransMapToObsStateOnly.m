%%%
%%  NOT completed yet, 2008.01.08
%
% The process of transfer the selected beacon in global map into the 
% observation relative to the final robot pose
%
% state only, no uncertainty involved
% to be used for NN by Euclidean distance 
%
% equations: (for each beacon)
%
%  x_new = (x_old-x_r)*cos(phi) + (y_old-y_r)*sin(phi)
%  y_new = -(x_old-x_r)*sin(phi) + (y_old-y_r)*cos(phi)
%
% by Shoudong -- 08/01/2008, draft
%
% input: 
%
% selected_beacon_state --- the selected beacon and the robot pose in the global map 
%      
%      robot is located at the last of the state vector
%  
% output: 
%
% beac --- the estimate of beacons in the map relative to the robot pose
%
% 


function beac = TransMapToObsStateOnly(selected_beacon_state)

globalmap_st = selected_beacon_state;

% change the order of robot and beacons -- put robot in front
num = size(globalmap_st,1);

slam_state = [globalmap_st(num-2:num,:);globalmap_st(1:num-3,:)];

% coordinate transformation -- relative to robot pose

nmb_beac = (size(slam_state,1)-3)/2; % number of beacons

% transfer the state vector and covariance matrix into the one in the 
 % coordinate system of the final robot position
 
beac = zeros(2*nmb_beac,1);
%pause

for i=1:nmb_beac;
    dx=slam_state(2*i+2)-slam_state(1);
    dy=slam_state(2*i+3)-slam_state(2);    
    phi=slam_state(3);
    beac(2*i-1:2*i) = [cos(phi)*dx+sin(phi)*dy; -sin(phi)*dx+cos(phi)*dy];
end

return

