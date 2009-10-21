%
% function: update step in map joining
% use sparse matrix representation for Jacobian
%
% last modified: 24/09/2009, Shoudong
%

function DoGlobalMapUpdate

global Params;
global Match;
global Est;


% disp(' *** entering DoGlobalMapUpdate');

observations = Est.St;

observation_cov = Est.P;

%% delete the poor observations

measuredBeacon = observations;

index_poor_obs = [];

match_matrix = Match.LocalMap;

obs_match_matrix = kron(match_matrix,[1;1]);

dim_obs_now = size(obs_match_matrix,1);

obs_match_matrix = [obs_match_matrix(dim_obs_now-1:dim_obs_now,:); obs_match_matrix(dim_obs_now,:);obs_match_matrix(1:dim_obs_now-2,:)];

for k=1:(size(observations,1)-3)/2
    if Match.LocalMap(k,3)==-1
        index_poor_obs = [index_poor_obs; 2*k+2; 2*k+3];

    end
end

measuredBeacon(index_poor_obs,:) = [];
observation_cov(index_poor_obs,:)=[];
observation_cov(:,index_poor_obs)=[];

obs_match_matrix(index_poor_obs,:) = [];


%% remember the whole observation -- for smoothing using least squares, 24/09/2009

% match_matrix = Match.LocalMap;

if Params.SmoothAfterUpdate>0 % if do smoothing

    % format of Est.AllObservations
    % [local map number, local_index, index_of_observation_pose, global_index,  ID, observation value]


    submap_observation = [measuredBeacon(:,1:2),-(Params.IndexSubmap-1)*ones(size(measuredBeacon,1),1),measuredBeacon(:,2),measuredBeacon(:,3:4)];
    submap_observation(1:3,4:5) = [-Params.IndexSubmap -Params.IndexSubmap; -Params.IndexSubmap -Params.IndexSubmap; -Params.IndexSubmap -Params.IndexSubmap];

    submap_observation(4:size(measuredBeacon,1),4)= obs_match_matrix(4:size(measuredBeacon,1),3);

    %pause

    Est.AllObservations = [Est.AllObservations; submap_observation];
    Est.AllObservationCovInv = sparse(blkdiag(Est.AllObservationCovInv,inv(observation_cov)));
    Est.dimObs = [Est.dimObs, size(measuredBeacon,1)];

end


%% compute jacobian of h(observation model) and the predicted measurement
% use sparse matrix representation

% format of measurement
% measuredBeacon = [xr;yr;phir;x1;y1;...,xm,ym] = H(X^G)+w;
% formulas
% xr = (xr_2-xr_1)cos(phir_1)+(yr_2-yr_1)sin(phir_1)
% yr = -(xr_2-xr_1)sin(phir_1)+(yr_2-yr_1)cos(phir_1)
% phir = phir_2-phir_1
% x1 = (x1_G-xr_1)cos(phir_1)+(y1_G-yr_1)sin(phir_1)
% y1 = -(x1_G-xr_1)sin(phir_1)+(y1_G-yr_1)cos(phir_1)


size_global_state = size(Est.StGlobal,1);

jh = sparse(size(measuredBeacon,1),size_global_state);

H_predict = zeros(size(measuredBeacon,1),1);

% get the values of each element

index_rob_1_global = find(Est.StGlobal(:,1)==-Params.IndexSubmap+1, 1, 'first');

xr_1_est = Est.StGlobal(index_rob_1_global,5);
yr_1_est = Est.StGlobal(index_rob_1_global+1,5);
phir_1_est = Est.StGlobal(index_rob_1_global+2,5);

index_rob_2_global = find(Est.StGlobal(:,1)==-Params.IndexSubmap, 1, 'first');

xr_2_est = Est.StGlobal(index_rob_2_global,5);
yr_2_est = Est.StGlobal(index_rob_2_global+1,5);
phir_2_est = Est.StGlobal(index_rob_2_global+2,5);

%% compute Jacobians
% the first three rows (robot) of jacobian
jh_rob_1 = [-cos(phir_1_est), -sin(phir_1_est), -(xr_2_est-xr_1_est)*sin(phir_1_est)+(yr_2_est-yr_1_est)*cos(phir_1_est);
    sin(phir_1_est), -cos(phir_1_est), -(xr_2_est-xr_1_est)*cos(phir_1_est)-(yr_2_est-yr_1_est)*sin(phir_1_est);
    0, 0, -1];
jh_rob_2 = [cos(phir_1_est), sin(phir_1_est), 0;
    -sin(phir_1_est), cos(phir_1_est), 0;
    0, 0, 1];

jh(1:3,index_rob_1_global:index_rob_1_global+2) = jh_rob_1;
jh(1:3,index_rob_2_global:index_rob_2_global+2) = jh_rob_2;

% the first three rows (robot) of predicted measurement

H_predict(1:3) = [(xr_2_est-xr_1_est)*cos(phir_1_est)+(yr_2_est-yr_1_est)*sin(phir_1_est);
    -(xr_2_est-xr_1_est)*sin(phir_1_est)+(yr_2_est-yr_1_est)*cos(phir_1_est);
    phir_2_est-phir_1_est];

% the other rows (beacons) of jacobian and predicted measurement

for i = 4:2:size(measuredBeacon,1)-1
    index_beacon_i = find(Est.StGlobal(:,1)==Match.LocalMap(measuredBeacon(i,2),3), 1, 'first');
    x1_G_est = Est.StGlobal(index_beacon_i,5);
    y1_G_est = Est.StGlobal(index_beacon_i+1,5);
    %pause

    % compute Jacobians
    jh_rob_1_i = [-cos(phir_1_est), -sin(phir_1_est), -(x1_G_est-xr_1_est)*sin(phir_1_est)+(y1_G_est-yr_1_est)*cos(phir_1_est);
        sin(phir_1_est), -cos(phir_1_est), -(x1_G_est-xr_1_est)*cos(phir_1_est)-(y1_G_est-yr_1_est)*sin(phir_1_est)];
    jh_beac_i = [cos(phir_1_est), sin(phir_1_est);
        -sin(phir_1_est), cos(phir_1_est)];

    jh(i:i+1,index_rob_1_global:index_rob_1_global+2) = jh_rob_1_i;
    jh(i:i+1,index_beacon_i:index_beacon_i+1) = jh_beac_i;

    % the other rows (beacons) of predicted measurement
    H_predict(i:i+1) = [(x1_G_est-xr_1_est)*cos(phir_1_est)+(y1_G_est-yr_1_est)*sin(phir_1_est);
        -(x1_G_est-xr_1_est)*sin(phir_1_est)+(y1_G_est-yr_1_est)*cos(phir_1_est)];
end

%% do update
% compute innovation
% here theta elements in z_new and hu is of [-pi pi], but z_new-hu could be
% out of that scope, so do wrap again
z_new_hu = measuredBeacon(:,4) - H_predict;
z_new_hu(3) = wrap(z_new_hu(3));


% update the information vector
Est.InfoVectorGlobal(:,5) = Est.InfoVectorGlobal(:,5) + jh'*inv(observation_cov)*(z_new_hu+jh*Est.StGlobal(:,5));
% use sparse matrix representation
Inf_new = jh'*sparse(inv(observation_cov))*jh;

% find the first non-zero element and the matrix Omega, to be used
% in DoComputeCholeskyFact
[I,J]=find(Inf_new);
Num_Diff_First=min(min(I),min(J));
Est.InfoMatrixNumDiffFirst = Num_Diff_First;
Est.InfoMatrixOmega = Inf_new(Num_Diff_First:size_global_state,Num_Diff_First:size_global_state);

% update the information matrix
Est.InfoMatrixGlobal = Est.InfoMatrixGlobal + Inf_new;

%%%%%%%%%%%%%
%% compute small information matrix if using AMD small reordering
%%%%%%%%%%%%%%%
if (Params.ReorderAMD > 0)
    match_localmap = Match.LocalMap;
    size_info_matrix_small=size(Est.InfoMatrixGlobalSmall,1);
    index_local_all=[];
    for i=1:size(Match.LocalMap,1)
        index_i=find(Est.StGlobalSmall(:,1)==Match.LocalMap(i,3),1,'first');
        index_local_all=[index_local_all;index_i];
    end
    index_previous_robot=find(Est.StGlobalSmall(:,1)==-Params.IndexSubmap+1,1,'first')  ;
    index_local_all=[index_local_all;index_previous_robot];
    size_ones=size(index_local_all,1);
    inf_new_small=ones(size_ones,size_ones);
    for i=1:size_ones
        inf_new_small(i,i)=100;
    end
    Est.InfoMatrixGlobalSmall(index_local_all,index_local_all)=inf_new_small;
    %     test = Est.StGlobalSmall
    %     test = full(Est.InfoMatrixGlobalSmall)
    %    pause
end
%%%%%%%%%%%%%
%% end compute small information matrix if using AMD reordering
%%%%%%%%%%%%%%%

return;






