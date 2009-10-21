% Compute complete Cholesky factorization of information matrix
%
% If not reordered -- incremental
%
% Use the previous Cholesky factorization to compute the current Cholesky
% factorization
%
% If reordered -- direct
%
% last modified, Shoudong -- 24/09/2009
%
%
function DoComputeCholeskyFact

global Params;
global Est;

% to record the time used for Compute complete Cholesky factorization
% timeStart = cputime;
tic;

% size of the matrices
n_new = size(Est.InfoMatrixGlobal,1);

% the first non-equal elements of the two information matrix
n_diff_first = Est.InfoMatrixNumDiffFirst;

size_L22=n_new-n_diff_first+1;
ratio = size_L22/n_new;



if Params.RecursiveCholesky ==1
    if Est.ReorderSubmapsSign == 1

        Est.InfoMatrixCholeskyFactor = chol(Est.InfoMatrixGlobal,'lower'); % direct complete cholesky factorization

        Est.ReorderSubmapsSign = 0;
        Est.SizeL22Ratio=[ Est.SizeL22Ratio; Params.IndexSubmap, 1];
    else % use iterative factorization when ratio <Params.L22Ratio
        Est.SizeL22Ratio=[ Est.SizeL22Ratio; Params.IndexSubmap, ratio];

        if ratio<Params.L22Ratio

            %%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%% use the complete Cholesky factorization of I_old
            %%%  to construct the complete Cholesky factorization of I_new
            L_22 = Est.InfoMatrixCholeskyFactor(n_diff_first:n_new,n_diff_first:n_new);
            % construct the new Cholesky factorization
            L_22_new = chol(Est.InfoMatrixOmega+L_22*L_22','lower');

            Est.InfoMatrixCholeskyFactor(n_diff_first:n_new,n_diff_first:n_new)= L_22_new;

        else
            Est.InfoMatrixCholeskyFactor = chol(Est.InfoMatrixGlobal,'lower'); % direct complete cholesky factorization
        end
    end
else
    Est.InfoMatrixCholeskyFactor = chol(Est.InfoMatrixGlobal,'lower'); % direct complete cholesky factorization
end


% to record the time used for preconditioning
%timeUsed = cputime - timeStart;
timeUsed = toc;
Est.timeUsedCholeskyFact = [Est.timeUsedCholeskyFact;  timeUsed];

return;
