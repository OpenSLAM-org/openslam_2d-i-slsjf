% %% function: Main loop for Iterative Sparse Local Submap Joining Filter (I-SLSJF), framework
% %
% % Only requirement: the robot end pose in the current local map is the robot
% % starting pose in the next local map
% %
% % papers:
% % (1) S. Huang, Z. Wang, G. Dissanayake, and U. Frese,
% %     Iterated SLSJF: A sparse local submap joining algorithm with improved consistency,
% %     2008 Australiasan Conference on Robotics and Automation. Canberra, December 2008.
% %     Available online: http://www.araa.asn.au/acra/acra2008/papers/pap102s1.pdf
% % (2) S. Huang, Z. Wang and G. Dissanayake. Sparse local submap joining filter for building large-scale maps.
% %     IEEE Transactions on Robotics, 2008, Vol. 24, No. 5, 1121-1130, October 2008.
% %
% % input: a sequence of local maps -- (X_1, P_1), ..., (X_p, P_p)
% %        each X_i contain the robot end pose and the position of local
% point features
% %
% %        now the second column of X_i is the estimate, the first column
% is the feature ID (the robot ID is 0) if they are available
% %
% %        the local map i is stored in localmap_i.mat
% %
% % output: the global map -- (Est.StGlobal, Est.InfoVectorGlobal, Est.InfoMatrixGlobal)
% %         Est.StGlobal contain the locations of the robot end poses and all the beacons
% %         a few columns are added in the global state vector
% %         [global_index, local map number, local_index, ID, estimate]
%          (for global_index and ID, negative number means robot pose, e.g. -2 means robot end pose in local map 2)
% %
% %         Est.InfoVectorGlobal is the information vector, it has the same format as Est.StGlobal
% %         Est.InfoMatrixGlobalI is the sparse information matrix
% %
% %         The whole covariance matrix is never computed or stored, the block diagonal of covairance matrix
% %         can be chosen to be computed at the very end for drawing covariance ellipses
% %
% %
% % last modified: 2009, October 10, Shoudong Huang
% %
% % Copyright: This code is written by Shoudong Huang
% % (sdhuang@eng.uts.edu.au) and Zhan Wang (zhwang@eng.uts.edu.au)
% % ARC Centre of Excellence for Autonomous Systems,
% % Faculty of Engineering and Information Technology,
% % University of Technology, Sydney, Australia.
% % The code may be used and modified for research purposes only with
% % acknowledgement of the authors and inclusion of this copyright information.
% % The authors allow the users to use and modify the source code for their own research. 
% % Any commercial application, redistribution, etc. has to be arranged
% % between users and authors individually.
% 
% 
% 
function MainLoop

clc;
clear all;
close all;

global Params;
global Est;
global Match;

% setup the parameters
DoSetupParam;

% check the parameter set up
if (Params.AssumeDataAssoc==1) && (Params.Simulation < 0)
    disp(' *** Sorry, you can not choose Assume Data Association');
    disp(' *** since you do not have the true feature index');
    disp(' *** Please reset the parameter!');
    return;
end

% to record the time used for fusing the first local map
timeStart = cputime;

%% load the first local map

% load local map Params.IndexSubmapStart
index = num2str(Params.IndexSubmapStart);

if (Params.Simulation == 1) % simulation 1
    filename = strcat('535_data_5_local_maps/localmap_', index);
elseif (Params.Simulation == 2) % simulation 2
    filename = strcat('8240_data_50_local_maps/localmap_', index);
elseif (Params.Simulation == 3) % simulation 3
    filename = strcat('35188_data_700_local_maps/localmap_', index);
elseif (Params.Simulation == 0) % Test your own local maps with gound truth
    filename = strcat('Test_your_local_maps_with_ground_truth/localmap_', index);
elseif (Params.Simulation == 4) % DLR Spatial Recognition data set 200 local maps
    filename = strcat('DLR_200_local_maps/localmap_', index);
elseif (Params.Simulation == 5) % Victoria park data set 200 local maps
    filename = strcat('VicPark_200_local_maps/localmap_', index);
elseif (Params.Simulation == 6) % Test your own local maps
    filename = strcat('Test_your_local_maps/localmap_', index);
elseif (Params.Simulation == 7) % DLR Spatial Recognition data set 3298 local maps
    filename = strcat('DLR_3298_local_maps/localmap_', index);
elseif (Params.Simulation == 8) % Victoria park data set 6898 local maps
    filename = strcat('VicPark_6898_local_maps/localmap_', index);
elseif (Params.Simulation == -1) % Test your own local maps without feature index
    filename = strcat('Test_your_local_maps_without_true_index/localmap_', index);
else
    disp(' *** wrong Params.Simulation ***');
    return;
end

load(filename);

%% if robot did not move in the local map, increase the 0 uncertainty a bit
if (localmap_P(1,1)<Params.StabNoise)||(localmap_P(2,2)<Params.StabNoise)||(localmap_P(3,3)<Params.StabNoise)
    % add a bit on P to make it non-singular
    localmap_P(1:3,1:3) = localmap_P(1:3,1:3) + Params.StabNoise*[1, 0, 0; 0, 1, 0; 0, 0, 1];
    %  pause
end

%% add two columns in the local map
% original state [beacon_index_global, estimate]
% is changed into
% [local map number, beacon_index_local,  beacon_index_global, estimate]

localmap_st = AddTwoColumnsToSubmap(localmap_st);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% change the 1st local map into global map
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

DoFuseFirstGlobalMap(localmap_st,localmap_P);

% consistency check if have ground truth
if(Params.Simulation >= 0)&&(Params.Simulation <= 3);
    % check the consistency of the global map
    DoCheckConsistency_SLSJF_each_step;
end

% to record the time used for fusing the first local map
timeUsed = cputime - timeStart;

Est.timeUsed = [Est.timeUsed; timeUsed];

% increase 1 to the submap index
Params.IndexSubmap = Params.IndexSubmap + 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% fuse all the other local maps into the global map
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

i=2;

while i<= Params.NumOfSubmap
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % fuse local map i into the global map
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % to record the time used for fusing local map i
    timeStart2 = cputime;

    % load local map i

    index = num2str(i-1+Params.IndexSubmapStart);

    if (Params.Simulation == 1) % simulation 1
        filename = strcat('535_data_5_local_maps/localmap_', index);
    elseif (Params.Simulation == 2) % simulation 2
        filename = strcat('8240_data_50_local_maps/localmap_', index);
    elseif (Params.Simulation == 3) % simulation 3
        filename = strcat('35188_data_700_local_maps/localmap_', index);
    elseif (Params.Simulation == 0) % Test your own local maps with gound truth
        filename = strcat('Test_your_local_maps_with_ground_truth/localmap_', index);
    elseif (Params.Simulation == 4) % DLR Spatial Recognition data set 200 local maps
        filename = strcat('DLR_200_local_maps/localmap_', index);
    elseif (Params.Simulation == 5) % Victoria park data set 200 local maps
        filename = strcat('VicPark_200_local_maps/localmap_', index);
    elseif (Params.Simulation == 6) % Test your own local maps
        filename = strcat('Test_your_local_maps/localmap_', index);
    elseif (Params.Simulation == 7) % DLR Spatial Recognition data set 3298 local maps
        filename = strcat('DLR_3298_local_maps/localmap_', index);
    elseif (Params.Simulation == 8) % Victoria park data set 6898 local maps
        filename = strcat('VicPark_6898_local_maps/localmap_', index);
    elseif (Params.Simulation == -1) % Test your own local maps without feature index
        filename = strcat('Test_your_local_maps_without_true_index/localmap_', index);
    else
        disp(' *** wrong Params.Simulation ***');
        return;
    end

    load(filename);

    %% if robot did not move in the local map, increase the 0 uncertainty a bit
    if (localmap_P(1,1)<Params.StabNoise)||(localmap_P(2,2)<Params.StabNoise)||(localmap_P(3,3)<Params.StabNoise)
        % add a bit on P to make it non-singular
        localmap_P(1:3,1:3) = localmap_P(1:3,1:3) + Params.StabNoise*[1, 0, 0; 0, 1, 0; 0, 0, 1];
        % pause
    end

    %% add two columns in the local map
    localmap_st = AddTwoColumnsToSubmap(localmap_st);

    Est.St = localmap_st;
    Est.P = localmap_P;

    %% fuse the local map into global map

    sprintf('*** fusing local map  %d', Params.IndexSubmap)

    DoFuseToGlobalMap

    % consistency check if have ground truth
    if(Params.Simulation >= 0)&&(Params.Simulation <= 3);
        DoCheckConsistency_SLSJF_each_step;
    end

    % to record the time used for fusing local map i
    timeUsed = cputime - timeStart2;
    Est.timeUsed = [Est.timeUsed;  timeUsed];

    %pause
    % increase 1 for the submap index
    Params.IndexSubmap = Params.IndexSubmap + 1;

    i=i+1;
end

%% total time used
TotaltimeUsed = cputime - timeStart

%% time used in fusing each local map
Time_used = Est.timeUsed;
% figure of the total time used%
figure(1)
plot(Time_used)
title(strcat('Time used for fusing each local map'))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% the sparse information matrix
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inf_matrix_final = sparse(Est.InfoMatrixGlobal);

Inf_matrix_final = Est.InfoMatrixGlobal;

figure(2)
spy(Inf_matrix_final);
title(strcat('The sparse information matrix'))

%% state dimension of global state vector, for drawing figures of time/dim
% State_Dim = Est.GlobalStDim;

if (Params.AssumeDataAssoc == 0) % if not assume data association
    Wrong_data_assoc = Est.WrongDataAssocNum
    Fail_joint_test = Est.JointTestFailNum
    Time_used_select_beacon_for_data_assoc = Est.timeUsedSelectBeaconForDataAssoc;
    Time_used_data_assoc = Est.timeUsedDataAssoc;
    Time_used_NN_data_assoc = Est.timeUsedNNDataAssoc;
    Loops_in_NN_data_assoc = Est.loopsInNNDataAssoc;
    Time_used_joint_comp_test = Est.timeUsedJointCompTest;

    % number of columns recovered in covariance matrix
    NumofColumnRecovered = Est.NumofColumnRecovered;
    Time_used_cov_recovery = [Est.timeUsedCovRecovery; max(Est.timeUsedCovRecovery)];
end

% time used in different operations
Time_used_Cholesky_Fact = Est.timeUsedCholeskyFact;
Time_used_state_recovery = Est.timeUsedStateRecovery;

if (Params.ReorderSubmaps == 1) % if reorder submaps
    Total_reorder=Est.ReorderTotalNum;
    Time_used_reordering = Est.timeUsedReordering;
end

if (Params.RecoverDiagCov==1)

    %%% recover the whole final covariance matrix by solving sparse linear equations
    DoFinalGlobalCovMatrixRecovery;

    Time_used_final_cov_recovery = Est.timeUsedFinalCovRecovery

end


%%%%%%%%%%%%%%%%%%%%%%%%%
%% draw the final map
%%%%%%%%%%%%%%%%%%%%%%%%%%

DoFigure;

% number of pass/fail in consistency check
if(Params.Simulation >= 0)&&(Params.Simulation <= 3);
    num_NEES_pass = Est.NEESpass
    num_NEES_fail = Est.NEESfail
end

%% save workspace
save('workspacefinal');

return;

