% global state recovery for map joining
%
% recover state vector before select beacon for data association, thus
% before recovery of covariance submatrix
%
% output: global map state vector 
%
% compute the state vector by solving linear equation Ix=i
% where i is the informatin vector
%
% use the complete Cholesky factorization to solve the equations
%
% Shoudong -- 2008.01.04
%

function DoGlobalStateRecovery

%global Params;
global Est;

% to record the time used for global state recovery
timeStart = cputime;

% recover the state vector by solving linear equation

n = size(Est.InfoMatrixGlobal,1);

b=Est.InfoVectorGlobal(:,5);
% 
% %%%%%%%%% use direct solution --- solve R*R'*x=b by R*y=b and R'*x=y
R = Est.InfoMatrixCholeskyFactor;

y=R\b;
X=R'\y;

%%%%%%%%%%%%%%%%%%%%


% state vector
Est.StGlobal(:,1:4) = Est.InfoVectorGlobal(:,1:4);
Est.StGlobal(:,5) = X;

%current_global_map = Est.StGlobal
%pause

% to record the time used for state recovery

timeUsed = cputime - timeStart;
Est.timeUsedStateRecovery = [Est.timeUsedStateRecovery;  timeUsed];

% to record the state dimension
Est.GlobalStDim = [Est.GlobalStDim; n];

return;
