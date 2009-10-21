% reorder the submaps in the global state vector and the global information
% vector --- to make the iterative Cholesky factorization more efficient
% and (possibly) reduce the fill-in
%
% Shoudong -- 2008.01.14,  AMD reordering 
%
%
%% AMD reordering only

function DoReorderSubmaps_AMD

global Params;
global Est;
%global Match;
%
%disp(' *** entering Reordering by AMD **** ');

% load workspacefinal 

% to record the time used for reordering
timeStart = cputime;

%% treat one robot and one beacon as a single element in the information matrix

% get the correspondence between the small state and big state

size_small = size(Est.StGlobalSmall,1);
small_big_correspondence = zeros(size_small,3);

add = 0; % the number of poses in front of k-th row
for k=1:size_small
    if Est.StGlobalSmall(k,1)<0
        small_big_correspondence(k,:) = [k, 2*k-1+add 3];
        add = add+1;
    else
        small_big_correspondence(k,:) = [k, 2*k-1+add 2];
    end
end


% to record the time used for getting small information matrix
timeStart_info_small = cputime;

%use the saved small information matrix
info_matrix_small=Est.InfoMatrixGlobalSmall;

% record the time used to get the small infomation matrix
timeUsed = cputime - timeStart_info_small;
Est.timeUsedReorderingGetInfoMatrixSmall = [Est.timeUsedReorderingGetInfoMatrixSmall;  timeUsed];
% pause

%% to record the time used for reordering the small information matrix
timeStart_reorder_small_matrix = cputime;

% reorder the small informaiton matrix
%pp=amd(info_matrix_small);
pp=symamd(info_matrix_small);

Est.InfoMatrixGlobalSmall=Est.InfoMatrixGlobalSmall(pp,pp);
Est.StGlobalSmall=Est.StGlobalSmall(pp,:);

%%% record the time used to get the small_state
timeUsed = cputime - timeStart_reorder_small_matrix;
Est.timeUsedReorderingReorderSmallMatrix = [Est.timeUsedReorderingReorderSmallMatrix;  timeUsed];
% pause

%% to record the time used for change the small reorder to big reorder
timeStart_reorder_small_to_big = cputime;

% to get big permutation
size_global_state = size(Est.StGlobal,1);
pp_big = zeros(size_global_state,1);

j=1;

for i=1:size_small
    i_big = small_big_correspondence(pp(i),2);
    pp_big (j) =  i_big;
    pp_big (j+1) = i_big+1;
    j = j+2;
    if small_big_correspondence(pp(i),3)==3
        pp_big (j) = i_big+2;
        j = j+1;
    end
end
 
Est.InfoMatrixGlobal = Est.InfoMatrixGlobal(pp_big,pp_big);

Est.InfoVectorGlobal = Est.InfoVectorGlobal(pp_big,:);

Est.StGlobal = Est.StGlobal(pp_big,:);

% test_global_state = Est.StGlobal
% pause
%%% record the time used to change small reorder to big reorder
timeUsed = cputime - timeStart_reorder_small_to_big;
Est.timeUsedReorderingSmallReorderToBig = [Est.timeUsedReorderingSmallReorderToBig;  timeUsed];
% pause

% remember the total number of reordering
Params.NumOfReorder = Params.NumOfReorder + 1;

% remember that a reorder just happen -- for selection of iterative
% Cholesky factorization or not
Est.ReorderSubmapsSign = 1;

timeUsed = cputime - timeStart;
Est.timeUsedReordering = [Est.timeUsedReordering;  timeUsed];

return;
