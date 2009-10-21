% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function -- joint compability test
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tardos, IJRR 
%
% by Shoudong (and Cindy) -- 1/03/2006
% 
% input: (1) current_match -- matrix of two columns, 
%                         first column: index of obs,
%                         second column: index of beac
%         (2) beac,beaccov -- beacon vector and covariance matrix
%         (3) obs,obscov -- observation vector and covariance matrix
%
% output: (1) pass (= 1 -- pass the test, = -1, not pass)
%         (2) mahadist_current_match -- the Mahalanobis distance 
%             of the two vectors
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% function -- joint compability test
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [pass,mahadist_current_match] = joint_compability_test(current_match,beac,beaccov,obs,obscov)

global Params; % need the confidence level for chi-square gate, e.g. 0.99 --- Params.ConfidenceJoint
global Est;

%disp(' *** entering joint compability test');

% to record the time used 
timeStart = cputime;

% get the corresponding rows from the two vectors --- obs and beac
row_obs=[];
row_beac=[];

for i=1:size(current_match,1)
    row_obs = [row_obs;2*current_match(i,1)-1;2*current_match(i,1)];
    row_beac = [row_beac;2*current_match(i,2)-1;2*current_match(i,2)];
end

% get the two smaller size vectors together with the covariance matrices
obs_select = obs(row_obs);
obscov_select = obscov(row_obs,row_obs);
beac_select = beac(row_beac);
beaccov_select = beaccov(row_beac,row_beac);

% compute Mahalanobis distance of the two vector
innov = beac_select-obs_select;
totalcov = beaccov_select + obscov_select;
mahadist_current_match  = sqrt(innov'*inv(totalcov)*innov);
% dimension of the innovation vector
dim=size(innov,1);

% compare the Mahalanobis distance with the gate 
mahadist_threshold = sqrt(chi2inv(Params.ConfidenceJoint,dim));
if mahadist_current_match  >= mahadist_threshold
%    disp(' *** current_match NOT pass the joint compability test');
    pass=-1;
    %pause
else
%    disp(' *** current_match pass the joint compability test'); 
    pass=1;
end

% to record the time used in joint compatibility test
timeUsed = cputime - timeStart;
Est.timeUsedJointCompTest = [Est.timeUsedJointCompTest;  timeUsed];

return

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% end function -- joint compability test
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        