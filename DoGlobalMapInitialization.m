%
% function: use robot starting point to initialize new beacons
%
% input:
%   newbeacon: new beacon measurements  [index_global Submap_num index_local ID x_hat y_hat]
%
%
% output:
% date: 12.9, zhan % Shoudong 2006.03.15

function DoGlobalMapInitialization(newbeacon, robot)

global Est;
global Params;

%disp(' *** entering DoGlobalMapInitialization');


x3_hat = newbeacon(1,5);    % coordinate of the beacon in robot coordinate system
y3_hat = newbeacon(1,6);
xr = robot(1);
yr = robot(2);
phir = robot(3);

% compute the new beacon position in global coordinate system

x3 = xr + x3_hat*cos(phir) - y3_hat*sin(phir);
y3 = yr + y3_hat*cos(phir) + x3_hat*sin(phir);

%pause

% initialized state vector
Est.StGlobalini = [Est.StGlobalini; newbeacon(1,1:4), x3; newbeacon(1,1:4), y3];
% compute Est.InfoVectorGlobal and Est.InfoMatrixGlobal
Est.StGlobal = [Est.StGlobal; newbeacon(1,1:4), x3; newbeacon(1,1:4), y3];

% add zeros to information vector, information matrix, and Cholesky
% factorization
Est.InfoVectorGlobal = [Est.InfoVectorGlobal; newbeacon(1,1:4), 0; newbeacon(1,1:4), 0];
inf_size=size(Est.InfoMatrixGlobal,1);
%pause
Est.InfoMatrixGlobal(inf_size+2,inf_size+2)=0;
Est.InfoMatrixCholeskyFactor(inf_size+2,inf_size+2)=0;

%%%%%%%%%%%%%
%% compute small information matrix if using AMD reordering
%%%%%%%%%%%%%%%
if (Params.ReorderAMD > 0)
    size_info_matrix_small=size(Est.InfoMatrixGlobalSmall,1);
    Est.InfoMatrixGlobalSmall(size_info_matrix_small+1,size_info_matrix_small+1)=100;
    Est.StGlobalSmall = [Est.StGlobalSmall; Est.StGlobal(inf_size+1,1:4)];
    %     test = Est.StGlobalSmall
    %     pause
end
%%%%%%%%%%%%%
%% end compute small information matrix if using AMD reordering
%%%%%%%%%%%%%%%


return;

