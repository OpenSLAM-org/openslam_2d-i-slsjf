%
% function: fuse the first local map into global map
%
% input: first localmap (X_1,P_1)
%
% output: the first global map (X,P)
% -- just change the order of robot and beacon
% (in order to get better sparseness in the information matrix)
%
% last modified: 22/09/2009 Shoudong

function DoFuseFirstGlobalMap(localmap_st,localmap_P)

global Params;
global Match;
global Est;


disp(' *** entering DoFuseFirstGlobalMap');

% two steps:
% (1) change the order of robot and features
% (2) compute Est.InfoVectorGlobal and Est.InfoMatrixGlobal

% disp('@@@@@@@@@@@@@@@@@@@@@     this is the first submap');

%% remember the whole observation -- for smoothing using least squares, 24/09/2009

if Params.SmoothAfterUpdate>0 % if do smoothing

    % format [local map number, local_index, index_of_observation_pose, global_index,  ID, observation value]
    first_observation = [localmap_st(:,1:2),zeros(size(localmap_st,1),1),localmap_st(:,2),localmap_st(:,3:4)];
    first_observation(1:3,4:5) = [-1 -1; -1 -1; -1 -1];
    %pause
    Est.AllObservations = first_observation;
    Est.AllObservationCovInv = inv(localmap_P);
    Est.dimObs = [size(localmap_st,1)];

end

%% add another column in the global state vector
% [global_index, local map number, local_index, ID, estimate]

sizeofSt = size(localmap_st,1);

state_slam = localmap_st;         % by shoudong
state_slam(1:3,2:3) = [zeros(3,1),-ones(3,1)];
state_slam = [state_slam(:,2),state_slam];
state_slam(1:3,1)=[-Params.IndexSubmap;-Params.IndexSubmap;-Params.IndexSubmap];
state_slam(1:3,2)=[-Params.IndexSubmap;-Params.IndexSubmap;-Params.IndexSubmap];
P_slam = localmap_P;

state_new = [state_slam(4:sizeofSt,:);state_slam(1:3,:)];

Est.StGlobal = state_new;



P_slam_row_exchange = [P_slam(4:sizeofSt,:);P_slam(1:3,:)];
P_new = [P_slam_row_exchange(:,4:sizeofSt),P_slam_row_exchange(:,1:3)];
Est.PGlobal = P_new;


%% initial state vector
Est.StGlobalini = state_new;
%pause


% information matrix
Info_Matrix_Global = inv(Est.PGlobal);
Est.InfoMatrixGlobal = sparse(Info_Matrix_Global);
%pause

%%%%%%%%%%%%%
%% compute small information matrix if using AMD reordering
%%%%%%%%%%%%%%%
if (Params.ReorderAMD > 0)
    size_info_matrix_small=(sizeofSt-1)/2;
    Est.InfoMatrixGlobalSmall=sparse(ones(size_info_matrix_small,size_info_matrix_small));
    for i=1:size_info_matrix_small
        Est.InfoMatrixGlobalSmall(i,i)=100;
    end
    %     test=Est.InfoMatrixGlobalSmall
    %     test_full=full(test)
    %     pause


    Est.StGlobalSmall = [];

    for i=1:(sizeofSt-1)/2
        Est.StGlobalSmall = [Est.StGlobalSmall; Est.StGlobal(2*i,1:4)];
    end
    %     test = Est.StGlobalSmall
    %     pause
end
%%%%%%%%%%%%%
%% end compute small information matrix if using AMD reordering
%%%%%%%%%%%%%%%


%%%%% compute the complete Cholesky factorization

Est.InfoMatrixCholeskyFactor = chol(Est.InfoMatrixGlobal)'; % complete Cholesky factorization
Est.timeUsedCholeskyFact = [Est.timeUsedCholeskyFact;  0]; % assume no time for the first Cholesky factorization


% information vector
Est.InfoVectorGlobal(:,5) = Est.InfoMatrixGlobal*Est.StGlobal(:,5);
Est.InfoVectorGlobal(:,1:4) = Est.StGlobal(:,1:4);

Est.StGlobalPrevious = Est.StGlobal; % keep this for checking whether to do smoothing or not


%% the Match matrix
% each row -- [submap num, index_submap, index_globalmap, ID]
match_matrix = [Params.IndexSubmap*ones(sizeofSt,1),Est.StGlobal(:,1),Est.StGlobal(:,1),Est.StGlobal(:,4)];
match_matrix(sizeofSt-2:sizeofSt,2) = [0;0;0];
%pause

% delete repeated rows
match_matrix(sizeofSt,:) = [];
ii = 1:2:sizeofSt-2;
match_matrix(ii,:) = [];
% match_matrix
% pause

% Match matrix -- correspondance from local map to the global map
Match.matrix = match_matrix;


%% prepare for data association if data association is not assumed
%if Params.AssumeDataAssoc==0 % do it anyway -- for reordering

% compute the radius of submap -- to simplify data association
radius = ComputeSubmapRadius(localmap_st,localmap_P);

Est.SubmapRadius = [Est.SubmapRadius; Params.IndexSubmap, radius];

%end


return;
