%
% function: check consistency of the global map
%
%
% date: 2009 Jan 29, Shoudong


function DoCheckConsistency_SLSJF_each_step

global Est;
global Params;

%% this is used to get the true robot final poses
if (Params.Simulation == 1) % simulation 1
    TotalNumOfSubmap=5; % 535 steps

    load 535_data_5_local_maps/store_robotTrue
    load 535_data_5_local_maps/store_beaconsTrue
elseif (Params.Simulation == 2) % simulation 2
    TotalNumOfSubmap=50; % 8240 steps

    load 8240_data_50_local_maps/store_robotTrue
    load 8240_data_50_local_maps/store_beaconsTrue
elseif (Params.Simulation == 3) % simulation 3
    TotalNumOfSubmap=700; % 35188 loops

    load 35188_data_700_local_maps/store_robotTrue
    load 35188_data_700_local_maps/store_beaconsTrue
elseif (Params.Simulation == 0) % your own local map with ground truth   

    load Test_your_local_maps_with_ground_truth/store_robotTrue
    load Test_your_local_maps_with_ground_truth/store_beaconsTrue
    load Test_your_local_maps_with_ground_truth/TotalNumOfSubmap
    
    TotalNumOfSubmap=TotalNumOfSubmap; % total number of submaps
    
end

Confidence_level_1 = 0.025;
Confidence_level_2 = 0.975;
Confidence_level = 0.99;

%get the robot true position from the data
robot_truth=[];

for i=1:TotalNumOfSubmap-1
    index = round((size(store_robotTrue,2)-1)/TotalNumOfSubmap*i);
    robot_truth = [robot_truth, store_robotTrue(:,index+1)];
end

robot_truth = [robot_truth, store_robotTrue(:,size(store_robotTrue,2))];

truth_beacon = store_beaconsTrue;

% check the consistency of the map
est_state = Est.StGlobal;
true_state=[];

size_state = size(Est.StGlobal,1);

i=1;

phi_index = [];

while i<size_state
    index=est_state(i,4);
    if index<0
        true_state = [true_state; robot_truth(1,-index); robot_truth(2,-index);robot_truth(3,-index)];
        phi_index = [phi_index;i+2];
        i=i+3;
    elseif index>0
        true_state = [true_state; truth_beacon(index,2); truth_beacon(index,3)];
        i=i+2;
    end
    %    pause
end

size_est=size(est_state);
size_true=size(true_state);

%pause

diff = est_state(:,5)-true_state;

% wrap the angles
for j=1:size(phi_index,1)
    jk=phi_index(j);
    diff(jk)=wrap(diff(jk));
end

test_state = diff'*Est.InfoMatrixGlobal*diff;

gate_1 = chi2inv(Confidence_level_1,size_state);
gate_2 = chi2inv(Confidence_level_2,size_state);
gate = chi2inv(Confidence_level,size_state);

%if (test_state>gate_1 && test_state<gate_2) % use two sides
if (test_state<gate) % use one side
    %    pass=1
    Est.NEESpass = Est.NEESpass+1;
    Est.NEES = [Est.NEES; Params.IndexSubmap, test_state, gate, 1];
else
    loop=Params.IndexSubmap;
    pass=0;
    max_diff=max(abs(diff));
    Est.NEESfail=Est.NEESfail+1;
    Est.NEES = [Est.NEES; Params.IndexSubmap, test_state, gate, 0];
    Est.failCount = Est.failCount +1;
end

return;


