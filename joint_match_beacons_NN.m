%%%% Match observations with beacons using Nearest Neighbour only, can be improved further
%%%% e.g. (1) JCBB
%         (2) other global data association appraoches
%
% Some parameters are problem dependent
%
% if no match but very close -- poor observation 20/12/2007
%
% Step 1:
% compute Mahalanobis distance and Euclidean distance between each observation/beacon pair
% find the nearest neighbours for both, decide the match based on the parameters
%
% criteria: (1)
%
% Step 2:
% Perform a joint compatibility test, remember the result, but do nothing further
%
% last modified by Shoudong -- 3/10/2009
%
% INPUT:
%
% obs --- observation vector (of a number of beacons)
% -- x and y coordinate 
%
% for example: in local map joining: (x_1, y_1, ..., x_m,y_m)
%
% obscov --- the covariance matrix of obs
%
% beac --- the estimate of beacons in the map
%          (***transferred into the observation space***)
%
% beaccov --- the covariance matrix of beac
%            (***also transferred into the observation space***)
%
% OUTPUT:
%
% the match between the obs and beac:
% match_matrix -- expressed by a two-column matrix, e.g.
% [1 2; 2 5; 3 -100; 4 -1] means
% 1 in obs matches 2 in beac, 2 in obs matches 5 in beac.
% 3 in obs is a new beacon,
% 4 in a poor observatoin --- obs does not match any beacon, but not far enough to be new beacon,
% this observation should be deleted (will not be used in the update)
%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% start the main match code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function match_matrix = joint_match_beacons_NN(beac,beaccov,obs,obscov) % nearest neighbour

global Params; % need the parameter--- problem dependent
global Est; % used to remember some information, not really necessary

%disp(' *** entering joint_match_beacons_NN');

% to record the time used
timeStart = cputime;

nmb_obs = size(obs,1)/2; % number of observations
nmb_beac = size(beac,1)/2; % number of beacons

%% to check the number of loops
loops=nmb_obs*nmb_beac;

% for record the matches
poor_obs=[];  % record the poor observations which will not be used in update
new_beacon=[]; % record the new beacons
nearest_match=[]; % record the nearest neighbour

%% compute threshold using problem dependent parameters
maha_dist_threshold = sqrt(chi2inv(Params.ConfidenceNN,2));
maha_dist_threshold_for_new_beacon = sqrt(chi2inv(Params.ConfidenceNewBeacon,2));

dist_threshold = Params.dist_threshold;
dist_threshold_for_match = Params.dist_threshold_for_match;
dist_threshold_for_new_beacon = Params.dist_threshold_for_new_beacon;

%% initialize the table of Euclidean distances and Mahalanobis distances

dist=zeros(nmb_obs,nmb_beac);
mahadist=zeros(nmb_obs,nmb_beac);
%% compute Euclidean distances and Mahalanobis distances tables
for j = 1:nmb_obs
    %obs_match_matrix (j,1) = j;
    %   num_matched_beacon = 0;
    x_obs = obs(j*2-1); % or x
    y_obs = obs(j*2);  % or y
    cov_obs = obscov(j*2-1:j*2,j*2-1:j*2);  % covariance matrix of the j-th obs
    for i = 1:nmb_beac
        x_beac = beac(i*2-1);
        y_beac = beac(i*2);
        cov_beac = beaccov(i*2-1:i*2,i*2-1:i*2);  % covariance matrix of the i-th beac
        % compute Mahalanobis distance from j-th obs to i-th beac
        innov = [x_beac-x_obs; y_beac-y_obs];
        dist(j,i) = sqrt(innov'*innov);
        totalcov = cov_beac+cov_obs;
        mahadist(j,i) = sqrt(innov'*inv(totalcov)*innov);
    end
end

%% use user defined criteria to decide the match

%%% for debug
% mahadist
% dist
%%% end for debug

% minimal distance from beacon to obs
min_maha_beac_to_obs = zeros(1,nmb_beac);
index_min_maha_beac_to_obs = zeros(1,nmb_beac);

min_dist_beac_to_obs = zeros(1,nmb_beac);
index_min_dist_beac_to_obs = zeros(1,nmb_beac);

for i=1:nmb_beac
    [min_maha_beac_to_obs(i), index_min_maha_beac_to_obs(i)] = min(mahadist(:,i)); % minimal Mahalanobis distance to beac i
    [min_dist_beac_to_obs(i), index_min_dist_beac_to_obs(i)] = min(dist(:,i)); % minimal Euclidean distance to beac i
end

% minimal distance from obs to beacon
min_maha_obs_to_beac = zeros(nmb_obs,1);
index_min_maha_obs_to_beac = zeros(nmb_obs,1);

min_dist_obs_to_beac  = zeros(nmb_obs,1);
index_min_dist_obs_to_beac = zeros(nmb_obs,1);

for j=1:nmb_obs

    [min_maha_obs_to_beac(j), index_min_maha_obs_to_beac(j)] = min(mahadist(j,:)); % minimal Mahalanobis distance to obs j
    [min_dist_obs_to_beac(j), index_min_dist_obs_to_beac(j)] = min(dist(j,:)); % minimal Euclidean distance to obs j

%     % for debug
%     if (Params.IndexSubmap == 17)||(Params.IndexSubmap == 51)||(Params.IndexSubmap == 49)||(Params.IndexSubmap == 79)...
%             ||(Params.IndexSubmap == 94)||(Params.IndexSubmap == 98)||(Params.IndexSubmap == 123)||(Params.IndexSubmap == 136)...
%             ||(Params.IndexSubmap == 4)||(Params.IndexSubmap == 25)
%         j=j
%         min_maha = min_maha_obs_to_beac(j)
%         index_min = index_min_maha_obs_to_beac(j)
%         maha_dist_threshold = maha_dist_threshold
%         maha_dist_threshold_for_new_beacon = maha_dist_threshold_for_new_beacon
% 
%         min_dist = min_dist_obs_to_beac(j)
%         index_min_dist = index_min_dist_obs_to_beac(j)
%         dist_threshold = dist_threshold
%         dist_threshold_for_match = dist_threshold_for_match
%         dist_threshold_for_new_beacon = dist_threshold_for_new_beacon
%         
%         index_i_j_maha = index_min_maha_beac_to_obs(index_min_maha_obs_to_beac(j))
%         index_i_j_dist = index_min_dist_beac_to_obs(index_min_maha_obs_to_beac(j))
%         %    pause
%     end
%     % end for debug

    %% use the parameters to decide the match -- many different ways to make the decision
    % Case 1: match nearest neighbour
    %         (1) both distances are less than threshold 
    %         (2) the two indices are the same
    %         (3) the obs j is also the closest point from the corresponding beac
    %
    if (min_maha_obs_to_beac(j) < maha_dist_threshold) && (min_dist_obs_to_beac(j)<dist_threshold)...
            && (index_min_maha_obs_to_beac(j)==index_min_dist_obs_to_beac(j))...
            && (index_min_maha_beac_to_obs(index_min_maha_obs_to_beac(j))==j)...
            && (index_min_dist_beac_to_obs(index_min_maha_obs_to_beac(j))==j)
      %  disp(' *** match nearest neighbour');
        nearest_match = [nearest_match; j index_min_maha_obs_to_beac(j) min_maha_obs_to_beac(j) dist(j,index_min_maha_obs_to_beac(j))]; % nearest neighbour
       % pause
    %% Case 2: match nearest neighbour according to Euclidean distance 
    %         (1) Euclidean distance is less than dist_threshold_for_match 
    %         (2) the two indices are the same
    %         (3) the obs j is also the closest point from the corresponding beac 
    elseif (min_dist_obs_to_beac(j) < dist_threshold_for_match) ...
            && (index_min_maha_obs_to_beac(j)==index_min_dist_obs_to_beac(j)) ...
            && (index_min_maha_beac_to_obs(index_min_maha_obs_to_beac(j))==j)...
            && (index_min_dist_beac_to_obs(index_min_maha_obs_to_beac(j))==j)
    %    disp(' *** Euclidean distance less than threshold');
        nearest_match = [nearest_match; j index_min_maha_obs_to_beac(j) min_maha_obs_to_beac(j) dist(j,index_min_maha_obs_to_beac(j))];
%        pause
     %% Case 3: new beacon
    %         (1) both distances are greater than threshold for new beacon
    elseif (min_maha_obs_to_beac(j) > maha_dist_threshold_for_new_beacon) && (min_dist_obs_to_beac(j) > dist_threshold_for_new_beacon) % new beacon
     %   disp(' *** new beacon');
        new_beacon = [new_beacon; j -100 mahadist(j,index_min_maha_obs_to_beac(j)) dist(j,index_min_maha_obs_to_beac(j))];
       % pause
    else
         %% Case 4: poor obervation -- will not use this obs information 
    %         all the other cases
     %   disp(' *** poor observation');
        poor_obs=[poor_obs; j -1 mahadist(j,index_min_maha_obs_to_beac(j)) dist(j,index_min_maha_obs_to_beac(j))];
      %  pause
    end
end

% % % for debug
% if (Params.IndexSubmap > 0)
%     nearest_match=nearest_match
%     poor_obs=poor_obs
%     new_beacon=new_beacon
%     pause
% %
% %  beac
% %  obs
% % % dist
% % nearest_match
% % new_beacon
% % poor_obs
% %  pause
%  end for debug

%% test the nearest neighbour result using joint compability test

match_matrix=[];

if size(nearest_match,1)==0 % no match at all
    % return the result -- only new beacon (and poor obs)
    if size(poor_obs,1)>0
        match_matrix = poor_obs(:,1:2);
    end
    if size(new_beacon,1)>0
        match_matrix = [match_matrix; new_beacon(:,1:2)];
    end
    return
end

% call the joint compability test
[pass,temp_maha] = joint_compability_test(nearest_match,beac,beaccov,obs,obscov);

% return the result

if size(new_beacon,1)>0
    if size(poor_obs,1)>0
        match_matrix_temp = [nearest_match(:,1:2);poor_obs(:,1:2);new_beacon(:,1:2)];
    else
        match_matrix_temp = [nearest_match(:,1:2);new_beacon(:,1:2)];
    end
else
    if size(poor_obs,1)>0
        match_matrix_temp = [nearest_match(:,1:2);poor_obs(:,1:2)];
    else
        match_matrix_temp = [nearest_match(:,1:2)];
    end
end

if pass > 0

    %  disp(' *** nearest neighbour result PASS joint compability test');
    %
    %     % check whether 2 obs match 1 beac
    %     wrong_match=0;
    %     for i=1:size(nearest_match,1)-1
    %         for j=i+1:size(nearest_match,1)
    %             if nearest_match(i,2)==nearest_match(j,2)
    %                 wrong_match=1; % two obs match one beac
    %             end
    %         end
    %     end
    %     if wrong_match == 0 % no repeated match
    %         % return the result -- matched beacon, new beacon
    %         match_matrix = sortrows(match_matrix_temp,1);   % reorder the rows
    %         return
    %     else
    %         disp(' *** But 2 obs match 1 beac in nearest neighbour result!');
    %         match_matrix = sortrows(match_matrix_temp,1);   % reorder the rows
    %         return
    %     end
else
    disp(' *** nearest neighbour result NOT PASS joint compability test');
    Est.JointTestFailNum = [Est.JointTestFailNum; Params.IndexSubmap];
end

match_matrix = sortrows(match_matrix_temp,1);   % reorder the rows


% to record the time used in NN data association
timeUsed = cputime - timeStart;
Est.timeUsedNNDataAssoc = [Est.timeUsedNNDataAssoc;  timeUsed];

Est.loopsInNNDataAssoc = [Est.loopsInNNDataAssoc;  loops];

return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% end of the joint match beacons code
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
