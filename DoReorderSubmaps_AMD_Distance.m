% reorder the submaps in the global state vector and the global information
% vector --- to make the iterative Cholesky factorization more efficient
% and (possibly) reduce the fill-in
%
%
% Shoudong -- 2008.01.08, Reordering by AMD and Distance -- see SLSJF paper
%

function DoReorderSubmaps_AMD_Distance

global Params;
global Est;
%global Match;

% disp(' *** entering Reordering by AMD and Distance **** ');


% to record the time used for reordering
timeStart = cputime;


state_global_whole = Est.StGlobal;
%size_global_state = size(state_global_whole,1);

size_global_small = size(Est.StGlobalSmall,1);


%%%%%%%%
% get the correspondence between the small state and big state

small_big_correspondence = zeros(size_global_small,3);

add = 0; % the number of poses in front of k-th row
for k=1:size_global_small
    if Est.StGlobalSmall(k,1)<0
        small_big_correspondence(k,:) = [k, 2*k-1+add 3];
        add = add+1;
    else
        small_big_correspondence(k,:) = [k, 2*k-1+add 2];
    end
end
%%%%%%%%

% global x y position of the final robot 
index_final_robot = find(state_global_whole(:,1)==-Params.IndexSubmap, 1, 'first');

x_r_final = state_global_whole(index_final_robot,5);
y_r_final = state_global_whole(index_final_robot+1,5);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% new way to get the small state %%%%%%
%%%%%%%%%%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% compute the distance from final robot to each submap
% if it small enough, then compute the distance from the features in that
% submap to final robot
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

state_global_small_distance = [];
index_distance = [];

% if Params.IndexSubmap>400
%  %   Params.dist_reorder = Params.IndexSubmap/300*15;
%      Params.dist_reorder = 20;
% end
    

for i=1:Params.IndexSubmap
    % find the origin of submap i 
    if i>1
        index_i = find(Est.StGlobal(:,1)==-i+1, 1, 'first');
        x_i = Est.StGlobal(index_i,5);
        y_i = Est.StGlobal(index_i+1,5);
    else
        x_i = 0;
        y_i = 0;
    end
    % distance from final robot to submap i
    dist_submap_i = sqrt((x_i-x_r_final)^2+(y_i-y_r_final)^2);
    % check whether the distance is small enough
    if dist_submap_i<Params.dist_reorder+Est.SubmapRadius(i,2)+Params.EstBias
        % find the features of this submap
        index = find(Est.StGlobalSmall(:,2)==i);
        if size(index,1)>0 % at least one feature in this submap
            for jj=1:size(index,1)
                index_big=small_big_correspondence(index(jj),2);
                x = state_global_whole(index_big,5);
                y = state_global_whole(index_big+1,5);
                dist = sqrt((x-x_r_final)^2+(y-y_r_final)^2);
                if dist<Params.dist_reorder
                    state_global_small_distance = [state_global_small_distance; Est.StGlobalSmall(index(jj),1:4), dist];
                    index_distance = [index_distance; index(jj)];
                end
            end
        end
    end
end

% put the last robot in state_global_small_distance
index_final_small = find(Est.StGlobalSmall(:,1)==-Params.IndexSubmap, 1, 'first');

state_global_small_distance = [state_global_small_distance; Est.StGlobalSmall(index_final_small,1:4), 0];
index_distance = [index_distance; index_final_small];

state_global_small_AMD = Est.StGlobalSmall;
% delete the index associated with distance
state_global_small_AMD(index_distance,:)=[];

size_small_AMD = size(state_global_small_AMD,1);

% reorder by distance
state_global_small_distance = sortrows(state_global_small_distance,-5);
       

%% use the saved small information matrix
info_matrix_small_all = Est.InfoMatrixGlobalSmall;


%% find the index of AMD poses and features
index_AMD = zeros(size_small_AMD,1);

for i=1:size_small_AMD
    index_i = find(Est.StGlobalSmall(:,1)==state_global_small_AMD(i,1),1,'first');
    index_AMD(i) = index_i;
end

info_matrix_small = info_matrix_small_all(index_AMD,index_AMD);



% to record the time used for reordering the small information matrix
timeStart_reorder_small_matrix = cputime;

%pp_AMD = amd(info_matrix_small);
pp_AMD = symamd(info_matrix_small);

% info_matrix_small_new = info_matrix_small(pp_AMD,pp_AMD);
% Cholesky_mmd = chol(info_matrix_small_new)';


size_small_distance = size(state_global_small_distance,1);
pp_distance = zeros(size_small_distance,1);

for i=1:size_small_distance
    iij = state_global_small_distance(i,1);
    jjj = find(Est.StGlobalSmall(:,1)==iij, 1, 'first');
    pp_distance(i) = jjj;
end

pp_AMD_real = index_AMD(pp_AMD,:);
pp = [pp_AMD_real;pp_distance];
%pause

%% reorder the small global state vector
Est.StGlobalSmall = Est.StGlobalSmall(pp,:);
Est.InfoMatrixGlobalSmall = Est.InfoMatrixGlobalSmall(pp,pp);


% to record the time used for change the small reorder to big reorder
timeStart_reorder_small_to_big = cputime;

% change the small reorder to big reorder
size_global_state = size(Est.StGlobal,1);
pp_big = zeros(size_global_state,1);

j=1;

for i=1:size_global_small
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

% remember the total number of reordering
Params.NumOfReorder = Params.NumOfReorder + 1;


% remember that a reorder just happen -- for selection of iterative
% Cholesky factorization or not
Est.ReorderSubmapsSign = 1;

timeUsed = cputime - timeStart;
Est.timeUsedReordering = [Est.timeUsedReordering;  timeUsed];
return;
