% use least squares to improve the consistency after update
%
% last modified: 24/09/2009, shoudong
%
% works ok when performing data association
%

function DoSmoothUsingLeastSquare

global Params;
global Match;
global Est;


disp(' *** entering DoSmoothUsingLeastSquare');
% match_localmap = Match.LocalMap;
%
% match_matrix = Match.matrix
%
% inf_matrix = Est.InfoMatrixGlobal;
%
% inf_vector = Est.InfoVectorGlobal;
%
% state_vector = Est.StGlobal;
%pause

size_global_state = size(Est.StGlobal,1);

% fuse all the local map information using the new linearization point
% tread all the local maps as one big observation

observations = Est.AllObservations;

% save observations observations

% st_global = Est.StGlobal
%
% save st_global st_global
%
% pause
dim_obs = Est.dimObs;
inv_observation_cov = Est.AllObservationCovInv;

% format of measurement -- local map 1
% measuredBeacon = [xr;yr;phir;x1;y1;...,xm,ym] = H(X^G)+w;
% formulas
% xr = xr_2
% yr = yr_2
% phir = phir_2
% x1 = x1_G
% y1 = y1_G

% format of measurement -- local map 2 to k
% measuredBeacon = [xr;yr;phir;x1;y1;...,xm,ym] = H(X^G)+w;
% formulas
% xr = (xr_2-xr_1)cos(phir_1)+(yr_2-yr_1)sin(phir_1)
% yr = -(xr_2-xr_1)sin(phir_1)+(yr_2-yr_1)cos(phir_1)
% phir = phir_2-phir_1
% x1 = (x1_G-xr_1)cos(phir_1)+(y1_G-yr_1)sin(phir_1)
% y1 = -(x1_G-xr_1)sin(phir_1)+(y1_G-yr_1)cos(phir_1)


% compute jacobian of h(observation model) and the predicted measurement
% use sparse matrix representation

% observations=observations
% pause
jh = sparse(size(observations,1),size_global_state);
H_predict = zeros(size(observations,1),1);

for index_local_map =1:Params.IndexSubmap

    % get the values of each element
    if index_local_map==1
        xr_1_est = 0;
        yr_1_est = 0;
        phir_1_est = 0;
        index_rob_1_global = 0;
    else
        index_rob_1_global = find(Est.StGlobal(:,1)==-index_local_map+1, 1, 'first');

        xr_1_est = Est.StGlobal(index_rob_1_global,5);
        yr_1_est = Est.StGlobal(index_rob_1_global+1,5);
        phir_1_est = Est.StGlobal(index_rob_1_global+2,5);
    end
    index_rob_2_global = find(Est.StGlobal(:,1)==-index_local_map, 1, 'first');

    xr_2_est = Est.StGlobal(index_rob_2_global,5);
    yr_2_est = Est.StGlobal(index_rob_2_global+1,5);
    phir_2_est = Est.StGlobal(index_rob_2_global+2,5);
    % use estimated value
    % the first three rows (robot) of jacobian
    if index_local_map>1
        jh_rob_1 = [-cos(phir_1_est), -sin(phir_1_est), -(xr_2_est-xr_1_est)*sin(phir_1_est)+(yr_2_est-yr_1_est)*cos(phir_1_est);
            sin(phir_1_est), -cos(phir_1_est), -(xr_2_est-xr_1_est)*cos(phir_1_est)-(yr_2_est-yr_1_est)*sin(phir_1_est);
            0, 0, -1];
        jh(sum(dim_obs(1:index_local_map-1))+1:sum(dim_obs(1:index_local_map-1))+3,index_rob_1_global:index_rob_1_global+2) = jh_rob_1;
    end
    jh_rob_2 = [cos(phir_1_est), sin(phir_1_est), 0;
        -sin(phir_1_est), cos(phir_1_est), 0;
        0, 0, 1];
    jh(sum(dim_obs(1:index_local_map-1))+1:sum(dim_obs(1:index_local_map-1))+3,index_rob_2_global:index_rob_2_global+2)= jh_rob_2;

    %     jh=jh
    %     pause

    % the first three rows (robot) of predicted measurement

    H_predict(sum(dim_obs(1:index_local_map-1))+1:sum(dim_obs(1:index_local_map-1))+3) = [(xr_2_est-xr_1_est)*cos(phir_1_est)+(yr_2_est-yr_1_est)*sin(phir_1_est);
        -(xr_2_est-xr_1_est)*sin(phir_1_est)+(yr_2_est-yr_1_est)*cos(phir_1_est);
        phir_2_est-phir_1_est];

    % the other rows (beacons) of jacobian and predicted measurement

    for i = 4:2:dim_obs(index_local_map)-1

        index_row = sum(dim_obs(1:index_local_map-1))+i;
        global_index = observations(index_row,4);

        index_beacon_i = find(Est.StGlobal(:,1)==global_index, 1, 'first');

        x1_G_est = Est.StGlobal(index_beacon_i,5);
        y1_G_est = Est.StGlobal(index_beacon_i+1,5);
        %pause

        % compute Jacobians using estimated value
        if index_local_map>1
            jh_rob_1_i = [-cos(phir_1_est), -sin(phir_1_est), -(x1_G_est-xr_1_est)*sin(phir_1_est)+(y1_G_est-yr_1_est)*cos(phir_1_est);
                sin(phir_1_est), -cos(phir_1_est), -(x1_G_est-xr_1_est)*cos(phir_1_est)-(y1_G_est-yr_1_est)*sin(phir_1_est)];
            jh(sum(dim_obs(1:index_local_map-1))+i:sum(dim_obs(1:index_local_map-1))+i+1,index_rob_1_global:index_rob_1_global+2) = jh_rob_1_i;
        end
        jh_beac_i = [cos(phir_1_est), sin(phir_1_est);
            -sin(phir_1_est), cos(phir_1_est)];

        jh(sum(dim_obs(1:index_local_map-1))+i:sum(dim_obs(1:index_local_map-1))+i+1,index_beacon_i:index_beacon_i+1) = jh_beac_i;

        % the other rows (beacons) of predicted measurement
        H_predict(sum(dim_obs(1:index_local_map-1))+i:sum(dim_obs(1:index_local_map-1))+i+1) = [(x1_G_est-xr_1_est)*cos(phir_1_est)+(y1_G_est-yr_1_est)*sin(phir_1_est);
            -(x1_G_est-xr_1_est)*sin(phir_1_est)+(y1_G_est-yr_1_est)*cos(phir_1_est)];
    end
end


% do update
% compute innovation

z_new_hu = observations(:,6) - H_predict;

% wrap the angles
for index_local_map =1:Params.IndexSubmap
    %     test = 3+sum(dim_obs(1:index_local_map-1))
    %     test_z=z_new_hu(3+sum(dim_obs(1:index_local_map-1)))
    %     pause
    z_new_hu(3+sum(dim_obs(1:index_local_map-1))) = wrap(z_new_hu(3+sum(dim_obs(1:index_local_map-1))));
end

% update the information vector
Est.InfoVectorGlobal(:,5) =  jh'*inv_observation_cov*(z_new_hu+jh*Est.StGlobal(:,5));
% use sparse matrix representation
Inf_new = jh'*inv_observation_cov*jh;


% update the information matrix
Est.InfoMatrixGlobal = Inf_new;

%% Cholesky Factorization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Est.ReorderSubmapsSign = 1; % do a direct Cholesky factorization

DoComputeCholeskyFact;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% global state vector revcovery
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
DoGlobalStateRecovery;

return;






