% function: set up all the necessary parameters for I-SLSJF
%           after this set up, run MainLoop.m
%
% last modified: 2009, Oct 5, Shoudong Huang
%
% Copyright: This code is written by Shoudong Huang
% (sdhuang@eng.uts.edu.au) and Zhan Wang (zhwang@eng.uts.edu.au)
% ARC Centre of Excellence for Autonomous Systems,
% Faculty of Engineering and Information Technology,
% University of Technology, Sydney, Australia.
% The code may be used and modified for research purposes only with
% acknowledgement of the authors and inclusion of this copyright information.
% The authors allow the users to use and modify the source code for their own research. 
% Any commercial application, redistribution, etc. has to be arranged
% between users and authors individually.

function DoSetupParam

global Params;
global Est;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% choose from different simulation or experiment data set
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if ID of the features are available in the local maps
% which can be used in Assume Data Associaiton and check the data association result
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Params.Simulation = 0; % Use your own local maps, with ground truth
% Params.Simulation = 1; % simulation data 1, 535 steps, with ground truth
% Params.Simulation = 2; % simulation data 2, 8240 steps, with ground truth
% Params.Simulation = 3; % simulation data 3, 35188 steps, with ground truth
% Params.Simulation = 4; % DLR data 200 local maps, no ground truth
% Params.Simulation = 5; % VicPark data 200 local maps, no ground truth
% Params.Simulation = 6; % Use your own local maps, no ground truth
% Params.Simulation = 7; % DLR data 3298 local maps, no ground truth
% Params.Simulation = 8; % VicPark data 6898 local maps, no ground truth
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% if ID of features are not available in the local maps
% Params.Simulation = -1; % Use your own local maps, no feature ID for checking data association

Params.Simulation = 5;

%% how many local maps to fuse
% total number of local maps in different data sets
% Params.NumOfSubmap = 200; % DLR or VicPark data 200 local maps
% Params.NumOfSubmap = 700; % 35188 steps
% Params.NumOfSubmap = 50; % 8240 steps
% Params.NumOfSubmap = 5; % 535 steps
% Params.NumOfSubmap = 3298; % DLR data 3298 local maps
% Params.NumOfSubmap = 6898; % VicPark data 6898 local maps

Params.NumOfSubmap = 200;

%% the index of the first local map to be fused
Params.IndexSubmapStart = 1; % for example, you can choose to fuse local map 11 to local map 20

%% perform data association or not
% Params.AssumeDataAssoc = 1; % assume data association
% Params.AssumeDataAssoc = 0; % not assume data association

Params.AssumeDataAssoc = 1;

%% display wrong data association or not
%Params.ShowWrongDataAssociation = 0; % do not display
%Params.ShowWrongDataAssociation = 1; % show wrong data association in figure

Params.ShowWrongDataAssociation = 1;

%% I-SLSJF or SLSJF
% Params.SmoothAfterUpdate = 0; % no smoothing, SLSJF
% Params.SmoothAfterUpdate = 1; % do least square whenever necessary, I-SLSJF
% Params.SmoothAfterUpdate = 2; % only do least square at the last step, result should be equivalent to I-SLSJF if data association is correct

Params.SmoothAfterUpdate = 2;

%% if choose I-SLSJF, need to choose the following two parameters
% threshold for least squares convergence
% if state difference is larger than this, then do smoothing

% Params.SmoothThreshold = 0.00000001; 
% Params.SmoothThreshold = 0.01; % 35188 loop, data association correct

Params.SmoothThreshold = 0.01;

% maximal iteration number for least squares smoothing
Params.SmoothMaxIterNum = 10;

%% recover the (block diagonal of) final covariance matrix or not
% Params.RecoverDiagCov = 0; % do not recover the final covariance, draw the estimate only in the map
% Params.RecoverDiagCov = 1; % recover the (block diagonal of) final covariance and draw the uncertainty ellipse in the final map

Params.RecoverDiagCov = 1;

%% some parameters for data association -- problem depenent
% these parameters are for our nearest neighbour matching algorithm, 
% different matching algorithm may need different parameters

if (Params.Simulation == 1) % simulation 1, 535 time steps 
    Params.EstBias = 0; % in meter, the maximal possible bias in the estimation, used to check the overlap between submaps etc.
    Params.MaxDistance = 3; % in meter, the maximal distance between two matched beacons
    Params.dist_threshold = 2; % in meter, threshold for Euclidean distance (to decide match by this and Mahanalobis distance threshold)
    Params.dist_threshold_for_match = 0.5; % in meter, smaller threshold for Euclidean distance (to decide match by itself)
    Params.dist_threshold_for_new_beacon = 2; % Euclidean distance threshold for new beacon
    Params.ConfidenceNN = 0.99; % confidence level for nearest neighbour -- to compute Mahanalobis distance threshold
    Params.ConfidenceNewBeacon = 0.7; % confidence level for new beacon -- to compute Mahanalobis distance threshold for new beacon
    Params.ConfidenceJoint = 0.9999; % confidence level for Joint Compatability test gate
elseif (Params.Simulation == 2) % simulation 2, 8240 time steps
    Params.EstBias = 1; 
    Params.MaxDistance = 3; 
    Params.dist_threshold = 2;
    Params.dist_threshold_for_match = 0.5; 
    Params.dist_threshold_for_new_beacon = 2;
    Params.ConfidenceNN = 0.99; 
    Params.ConfidenceNewBeacon = 0.7;
    Params.ConfidenceJoint = 0.9999; 
elseif (Params.Simulation == 3) % simulation 3, 35188 time steps
    Params.EstBias = 8;
    Params.MaxDistance = 3;
    Params.dist_threshold = 2.135;
    Params.dist_threshold_for_match = 0.5;
    Params.dist_threshold_for_new_beacon = 2; 
    Params.ConfidenceNN = 0.99; %
    Params.ConfidenceNewBeacon = 0.76;
    Params.ConfidenceJoint = 0.9999; %
elseif (Params.Simulation == 0) % test your own local maps, with ground truth
    Params.EstBias = 0; %
    Params.MaxDistance = 3;
    Params.dist_threshold = 2;
    Params.dist_threshold_for_match = 0.5;
    Params.dist_threshold_for_new_beacon = 2;
    Params.ConfidenceNN = 0.99; %
    Params.ConfidenceNewBeacon = 0.7;
    Params.ConfidenceJoint = 0.9999; % 
elseif (Params.Simulation == 4) % DLR Spatial Recognition data set, 200 local maps
    Params.EstBias = 5; % original
    Params.MaxDistance = 4; % original
  %% try larger value to see the problem of closing the large loop
  %  Params.EstBias = 50; 
  %  Params.MaxDistance = 50; 
    Params.dist_threshold = 0.29;
    Params.dist_threshold_for_match = 0.29;
    Params.dist_threshold_for_new_beacon = 0.5;
    Params.ConfidenceNN = 0.995;
    Params.ConfidenceNewBeacon = 0.7;
    Params.ConfidenceJoint = 0.9999;
elseif (Params.Simulation == 5) % Victoria park data set, 200 local maps
      Params.EstBias = 30; 
    Params.MaxDistance = 10;
    Params.dist_threshold = 3.51;
        Params.dist_threshold = 5;
    Params.dist_threshold_for_match = 0.8;
    Params.dist_threshold_for_new_beacon = 1.56;
      Params.ConfidenceNN = 0.995; %
      Params.ConfidenceNewBeacon = 0.97;
    Params.ConfidenceJoint = 0.9999;
elseif (Params.Simulation == 6) % test your own local maps, without ground truth
    Params.EstBias = 5; 
    Params.MaxDistance = 3;
    Params.dist_threshold = 10;
    Params.dist_threshold_for_match = 0.6;
    Params.dist_threshold_for_new_beacon = 2;
    Params.ConfidenceNN = 0.99; 
    Params.ConfidenceNewBeacon = 0.7;
    Params.ConfidenceJoint = 0.9999;
elseif (Params.Simulation == 7) % DLR Spatial Recognition data set 3298 local maps
    Params.EstBias = 5;
    Params.MaxDistance = 2.5;
    Params.dist_threshold = 0.39;
    Params.dist_threshold_for_match = 0.327;
    Params.dist_threshold_for_new_beacon = 0.3;
    Params.ConfidenceNN = 0.995;
    Params.ConfidenceNewBeacon = 0.7;
    Params.ConfidenceJoint = 0.9999;
elseif (Params.Simulation == 8) % Victoria park data set, 6898 local maps
       Params.EstBias = 5; 
    Params.MaxDistance = 10;
    Params.dist_threshold = 3.51;
        Params.dist_threshold = 5;
    Params.dist_threshold_for_match = 0.8;
    Params.dist_threshold_for_new_beacon = 1.4225;
     Params.ConfidenceNN = 0.995; 
      Params.ConfidenceNewBeacon = 0.97;
    Params.ConfidenceJoint = 0.9999;
elseif (Params.Simulation == -1) % test your own local maps, without feature index
    Params.EstBias = 0; 
    Params.MaxDistance = 3;
    Params.dist_threshold = 2;
    Params.dist_threshold_for_match = 0.5;
    Params.dist_threshold_for_new_beacon = 2;
    Params.ConfidenceNN = 0.99; 
    Params.ConfidenceNewBeacon = 0.7;
    Params.ConfidenceJoint = 0.9999; % 
end

%%%% for compute submap radius
Est.SubmapRadius = []; % to record the radius of a local map,
Params.SigmaMultiplier = 3; % 3 sigma bound for computing the radius of submap

%%% record information of data association
if (Params.AssumeDataAssoc == 0) % if not assume data association
    %%%% for beacon selection in data association
    Est.GlobalRobotUncertainty = [1, 0]; % to record the robot start pose xy uncertainty in global map,

    Est.WrongDataAssocNum = []; % record the total number of wrong data associations
    Est.JointTestFailNum = []; % record the total number of joint compatability test fails
end

%%%%% for recursive Cholesky factorization or not
%Params.RecursiveCholesky = 0; % not recursive Cholesky factorization
%Params.RecursiveCholesky = 1; % recursive Cholesky factorization

Params.RecursiveCholesky = 1;
Est.SizeL22Ratio =[];
Params.L22Ratio = 0.05; % to decide when to use iterative factorization


%% for robot pose estimate and uncertainty
Est.GlobalRobotPose = []; % to record the global positions of robot poses at the time that local map is fused
Est.GlobalRobotPoseUnc = []; % to record the uncertainty of the robot poses

%% for reorder the state vector or not
%Params.ReorderSubmaps = 0; % not reorder submaps
Params.ReorderSubmaps = 1; % reorder submaps

if (Params.ReorderSubmaps == 1) % if reorder submaps

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %% chose one from the three options for reordering
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %%%% option 1: reorder by AMD only
    % Params.ReorderAMD = 1; % reorder by AMD only
    %%% option 2: reorder by AMD and distance
    % Params.ReorderAMD = 2; % reorder by distance plus AMD (SLSJF paper)
    %

    Params.ReorderAMD = 2;

    if (Params.ReorderAMD == 1)

        Params.RecursiveCholesky = 0; % not recursive Cholesky factorization
        Params.ReorderAMDFrequency = 10; % how often to do the reordering

    elseif (Params.ReorderAMD == 2)

        Params.dist_reorder = 15; % the distance for reorder submaps,
        Params.MaxL22dimension = 400;
        %
    else
        disp(' *** The incorrect option for reordering is chosen **** ');
        return
    end

    % to remember the number of reordering
    Params.NumOfReorder = 0;

    Est.ReorderSubmapsSign = 0; % sign for reorder submaps, used in Cholesky factorization
    Est.ReorderTotalNum = []; % total number of reordering
end



%% for record the time used
Est.timeUsed = [];
Est.timeUsedSelectBeaconForDataAssoc = [];
Est.timeUsedDataAssoc = [];
Est.timeUsedNNDataAssoc = [];
Est.loopsInNNDataAssoc = [];
Est.timeUsedJointCompTest = [];
Est.timeUsedGlobalMapUpdate = [];
Est.GlobalStDim = [];

Est.NumofColumnRecovered = [];
Est.timeUsedCovRecovery = [];

Est.timeUsedCholeskyFact = [];
Est.timeUsedStateRecovery = [];

if (Params.ReorderSubmaps == 1) % if reorder state vector
    Est.timeUsedReordering = [];

    Est.timeUsedReorderingGetInfoMatrixSmall=[];
    Est.timeUsedReorderingReorderSmallMatrix=[];
    Est.timeUsedReorderingSmallReorderToBig=[];
end

Params.IndexSubmap = 1;     % index of current submap, start from 1, can not be others

% stabilizing noise added to robot uncertainty if robot does not move
Params.StabNoise = 1e-6;

%% use to record the consistency check result using NEES
Est.NEESpass = 0;
Est.NEESfail = 0;
Est.NEES = [];
Est.failCount = 0;

return;
