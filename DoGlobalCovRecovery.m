% covariance sub-matrix recovery for map joining
%
% recover this after select beacon for data association
%
% output: a few columns of global map covariance matrix
%
% compute the column of covariance matrix by solving linear equations Ix=e_i
% where e_i is the unit vector
%
%
% use the complete Cholesky factorization -- only one iteration to solve
% the equations
%
% Shoudong -- 2008.01.04
%
%
function DoGlobalCovRecovery

% global Params;
global Est;

% to record the time used for covariance recovery
timeStart = cputime;
%pause


%%% dimension of linear equation
n = size(Est.InfoMatrixGlobal,1);

% recover the covariance matrix corresponding to the selected beacons
% obtain a number of columns of the whole P by solving linear equations Ix=b=e_i
index=Est.SelectedBeaconForDataAssociation(:,1);

%% only the selected beacons is needed
Est.StGlobalSelected = Est.StGlobal(index,:);

% selected = Est.StGlobalSelected
%
% pause
%pause


% %%%%%%%%% use direct solution --- solve R*R'*x=b by R*y=b and R'*x=y
identity=speye(n); % a sparse identity matrix, a lot faster than sparse(eye(n)) !!

b=identity(1:n,index');

R = Est.InfoMatrixCholeskyFactor;

y=R\b;
X=R'\y;
P_Global=X;


% only need the selected rows
Est.PGlobalSelected = P_Global(index,:);
% make it symmetric -- numerically more stable
Est.PGlobalSelected = full((Est.PGlobalSelected+Est.PGlobalSelected')/2);


Est.NumofColumnRecovered = [Est.NumofColumnRecovered; size(Est.PGlobalSelected,1)];

timeUsed = cputime - timeStart;
Est.timeUsedCovRecovery = [Est.timeUsedCovRecovery;  timeUsed];

return;
