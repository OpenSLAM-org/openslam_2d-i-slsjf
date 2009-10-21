% The process of transfer the local map into the 
% map relative to the final robot location
%
% equations: (for each beacon)
%
%  x_new = (x_old-x_r)*cos(phi) + (y_old-y_r)*sin(phi)
%  y_new = -(x_old-x_r)*sin(phi) + (y_old-y_r)*cos(phi)
%
% by Shoudong -- 14/03/2006
%
% input: 
%
% slam_state --- the estimate of the robot and beacons in the localmap 
%      (relative to the robot initial location when start the localmap)
% the first column is the index (index of robot is 0)
%
% state_cov --- the corresponding covariance matrix 
% 
% output: 
%
% beac --- the estimate of beacons in the map relative to the robot
% beaccov --- the corresponding covariance matrix 
%
% 

function [beac,beaccov] = Translocalmaprobot(slam_state,state_cov)

%disp(' *** trasfering the localmap into map relative to final robot location');

nmb_beac = (size(slam_state,1)-3)/2; % number of beacons

% transfer the state vector and covariance matrix into the one in the 
 % coordinate system of the final robot position
 
beac = zeros(2*nmb_beac,1);
%pause
JH = zeros(2*nmb_beac,2*nmb_beac+3);

for i=1:nmb_beac;
    dx=slam_state(2*i+2)-slam_state(1);
    dy=slam_state(2*i+3)-slam_state(2);    
    phi=slam_state(3);
    beac(2*i-1:2*i) = [cos(phi)*dx+sin(phi)*dy; -sin(phi)*dx+cos(phi)*dy];
    JH(2*i-1:2*i,1:3)=[-cos(phi) -sin(phi) -sin(phi)*dx+cos(phi)*dy;
        sin(phi) -cos(phi) -cos(phi)*dx-sin(phi)*dy;]; 
    JH(2*i-1:2*i,2*i+2:2*i+3)=[cos(phi) sin(phi); -sin(phi) cos(phi)]; 
end

beaccov=JH*state_cov*JH';

return

