% function:  fuse the current (not the first) local map into global map
% EIF plus smoothing -- I-SLSJF
%
% input: the current local map
% the current global map is (Est.InfoVectorGlobal, Est.InfoMatrixGlobal)
%
% output: the new global map
% (information matrix, information vector, state estimate)
%
% date: 2005.9.13, zhan % last modified by Shoudong  24/09/2009

function DoFuseToGlobalMap

global Params;
global Match;
global Est;

% disp(' *** entering DoFuseToGlobalMap');

% compute the radius of submap -- to simplify data association
radius = ComputeSubmapRadius(Est.St,Est.P);
Est.SubmapRadius = [Est.SubmapRadius; Params.IndexSubmap, radius];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% prepare for data association if data association is not assumed
% (1) select beacon for data association
% (2) recover the covariance submatrix
% (3) record the global robot pose uncertainty
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
if Params.AssumeDataAssoc==0 %
    if Params.IndexSubmap==2 % select all beacons when current submap is submap 2
        Est.StGlobalSelected = Est.StGlobal;
        Est.PGlobalSelected = Est.PGlobal;
        index_last_robot=size(Est.StGlobal,1);
        Est.SelectedBeaconForDataAssociation = [[1:index_last_robot]',Est.StGlobal];
    end

    if Params.IndexSubmap>2
        % select the possible matched beacons
        DoSelectBeaconForDataAssociation;
        % recover the correponding covariance sub-matrix
        DoGlobalCovRecovery;        
    end

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % compute the robot start pose x y position uncertainty in global map,
    % used in beacon selection for data association
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    index_last_robot=size(Est.PGlobalSelected,1); % the robot pose is always at the last
    P_robot_xy = Est.PGlobalSelected(index_last_robot-2:index_last_robot-1,index_last_robot-2:index_last_robot-1);

    uncertainty = max(eig(sqrtm(P_robot_xy)))*Params.SigmaMultiplier;
    %     if uncertainty>1
    %         uncertainty
    %         % pause
    %     end
    Est.GlobalRobotUncertainty = [Est.GlobalRobotUncertainty; Params.IndexSubmap, uncertainty];

    %     rob_unc = Est.GlobalRobotUncertainty
    %     pause


    % store the robot pose estimate and uncertainty while fusing the local map
    % use it to compare with EKF map joining result --- to make a fair
    % comparison
    x_r = Est.StGlobalSelected(index_last_robot-2,5);
    y_r = Est.StGlobalSelected(index_last_robot-1,5);
    phi_r = Est.StGlobalSelected(index_last_robot,5);

    Est.GlobalRobotPose = [Est.GlobalRobotPose; Params.IndexSubmap-1, x_r, y_r, phi_r];
    %test = Est.GlobalRobotPose
    %pause

    p_xx = Est.PGlobalSelected(index_last_robot-2,index_last_robot-2);
    p_yy = Est.PGlobalSelected(index_last_robot-1,index_last_robot-1);
    p_phiphi = Est.PGlobalSelected(index_last_robot,index_last_robot);

    Est.GlobalRobotPoseUnc = [Est.GlobalRobotPoseUnc; Params.IndexSubmap-1, p_xx, p_yy, p_phiphi];
end

% submap_index = Params.IndexSubmap;

if(1)
    %% data association
    if Params.AssumeDataAssoc==0 %
        DoGlobalDataAssociation;
    elseif Params.AssumeDataAssoc==1 %
        DoGlobalDataAssociationAssumed;
    else
        disp('@@@@ Data Association parameter is not properly selected!');
    end
end


%% for testing only, temporarily use 2009 Oct 1
if(0)

    %% data association
    if Params.AssumeDataAssoc==0 %
%         if (Params.IndexSubmap == 280)||(Params.IndexSubmap == 574)||(Params.IndexSubmap == 766)...
%                 ||(Params.IndexSubmap == 767)||(Params.IndexSubmap == 843)||(Params.IndexSubmap == 878)...
%                 ||(Params.IndexSubmap == 879)||(Params.IndexSubmap == 881)||(Params.IndexSubmap == 1033)...
%                 ||(Params.IndexSubmap == 1034)...
%                 ||(Params.IndexSubmap == 1148)...
%                 ||(Params.IndexSubmap == 1177)||(Params.IndexSubmap == 1178)||(Params.IndexSubmap == 1183)...
%                 ||(Params.IndexSubmap == 2139)||(Params.IndexSubmap == 2557)...
%                 ||(Params.IndexSubmap == 2565)||(Params.IndexSubmap == 2937)...
%                 ||(Params.IndexSubmap == 2945)||(Params.IndexSubmap == 3016)...
%                 ||(Params.IndexSubmap == 3017)||(Params.IndexSubmap == 3203)...
%                 ||(Params.IndexSubmap == 3209)
                            if(Params.IndexSubmap <=3204)
            DoGlobalDataAssociationAssumed;
        else
            DoGlobalDataAssociation;
        end
    elseif Params.AssumeDataAssoc==1 %
        DoGlobalDataAssociationAssumed;
    else
        disp('@@@@ Data Association parameter is not properly selected!');
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%
% map fusion
%%%%%%%%%%%%%%%%%%%%%%%%%

% to record the time used for global state recovery
timeStart = cputime;

DoGlobalMapObservation;

%% to record the time used in Global map update
timeUsed = cputime - timeStart;
Est.timeUsedGlobalMapUpdate = [Est.timeUsedGlobalMapUpdate;  timeUsed];


% record the match matrix
Match.matrix = [Match.matrix; Match.LocalMap];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% reordering the global state vector
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if(Params.ReorderSubmaps==1) % if choose to do reordering
    if (Params.ReorderAMD == 1)
        %% reorder frequency
        if (Params.IndexSubmap/Params.ReorderAMDFrequency==round(Params.IndexSubmap/Params.ReorderAMDFrequency))
            %%% AMD reordering
            DoReorderSubmaps_AMD
            Est.ReorderTotalNum = [Est.ReorderTotalNum; Params.IndexSubmap]; % total number of reordering
        end
    elseif (Params.ReorderAMD == 2)
        % reorder by distance and AMD
        % if the lower-right submatrix L_22 larger than
        % Params.MaxL22dimension dim (here Est.InfoMatrixNumDiffFirst is obtained in DoGlobalUpdate.m)

        % if Est.InfoMatrixNumDiffFirst < size(Est.InfoVectorGlobal,1)-1/10*size(Est.InfoVectorGlobal,1)
        %            if Est.InfoMatrixNumDiffFirst < size(Est.InfoVectorGlobal,1)-max(1/10*size(Est.InfoVectorGlobal,1),Params.MaxL22dimension)
        if Est.InfoMatrixNumDiffFirst < size(Est.InfoVectorGlobal,1)-Params.MaxL22dimension
            %                  if Est.InfoMatrixNumDiffFirst < size(Est.InfoVectorGlobal,1)-100
            %              Num_Diff_First=Est.InfoMatrixNumDiffFirst
            %              size_global_L22=size(Est.InfoVectorGlobal,1)-Params.MaxL22dimension
            %% reordering by distance and AMD
            DoReorderSubmaps_AMD_Distance

            Est.ReorderTotalNum = [Est.ReorderTotalNum; Params.IndexSubmap]; % total number of reordering
        end
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Cholesky Factorization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
DoComputeCholeskyFact;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% global state vector revcovery
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Est.StGlobalPrevious = Est.StGlobal; % keep this for checking whether to do smoothing or not

DoGlobalStateRecovery;

%% do a smooth using least square to improve consistency
if (Params.SmoothAfterUpdate == 1) % smooth whenever necessary
    diff_state = Est.StGlobal(:,5)-Est.StGlobalPrevious(:,5);
    %           max_diff=max(abs(diff_state))
    %           pause
    count = 0;
    while max(abs(diff_state))>Params.SmoothThreshold && count<Params.SmoothMaxIterNum
        Est.StGlobalPrevious = Est.StGlobal; % keep this for checking whether to do smoothing or not
        DoSmoothUsingLeastSquare;

        %  local_map_index = Params.IndexSubmap
        count = count+1;
        diff_state = Est.StGlobal(:,5)-Est.StGlobalPrevious(:,5);
        %         max_diff=max(abs(diff_state))
        %         pause
    end
elseif (Params.SmoothAfterUpdate == 2)
    if (Params.IndexSubmap>Params.NumOfSubmap-1) % smooth the last step only

        diff_state = Est.StGlobal(:,5)-Est.StGlobalPrevious(:,5);

        count = 0;
        while max(abs(diff_state))>Params.SmoothThreshold && count<Params.SmoothMaxIterNum
            Est.StGlobalPrevious = Est.StGlobal; % keep this for checking whether to do smoothing or not
            DoSmoothUsingLeastSquare;

            count = count+1;
            diff_state = Est.StGlobal(:,5)-Est.StGlobalPrevious(:,5);

        end
    end
end


return;

