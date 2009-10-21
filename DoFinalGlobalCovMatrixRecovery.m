% final covariance matrix recovery for map joining
%
% output: final global map covariance matrix (only diagonal elements)
%
% compute the covariance matrix by solving linear equations Ix=e_i
% where e_i is the unit vector
%
% use the complete Cholesky factorization
%
% Shoudong -- 2008. Jan 11
%
% result -- correct
%
function DoFinalGlobalCovMatrixRecovery

%global Params;
global Est;

% to record the time used for covariance matrix recovery
timeStart = cputime;
%pause

%%% dimension of linear equation
n = size(Est.InfoMatrixGlobal,1);

R = Est.InfoMatrixCholeskyFactor;

%%%%%%%%%%
% compute the covaiance matrix by solving n linear equations

%identity matrix
%identity=speye(n);

% %%%% recover the whole covariance matrix, will run out of memory! 
% %%%% not necessary for the map figure, 2008.1.22
% %%%% the diagonal elements are enough
% 
% % use direct solution --- solve R*R'*x=b by R*y=b and R'*x=y
% y=R\identity;
% X=R'\y;
% Est.PGlobal=X;

%%%  only save the diagnal elements of covariance matrix. 2008.1.22, OK!

% identity matrix
identity=speye(n); % sparse identity matrix

% compute the covaiance matrix by solving n linear equations
Est.PGlobalDiag = sparse(n,n);

% start to solve n linear equations Ix=b=e_i
for i=1:n
    %    i
    % right hand side
    b=identity(:,i);
    % %%%%%%%%% use direct solution --- solve R*R'*x=b by R*y=b and R'*x=y
    y=R\b;
    X=R'\y;

    %% get the diagonal (3 lines) value for Est.PGlobal
    if i==1
        Est.PGlobalDiag(i:i+1,i)=X(i:i+1);
    elseif i==n
        Est.PGlobalDiag(i-1:i,i)=X(i-1:i);
    else
        Est.PGlobalDiag(i-1:i+1,i)=X(i-1:i+1);
    end

end

% record the time used
timeUsed = cputime - timeStart;
Est.timeUsedFinalCovRecovery = timeUsed;

return;
