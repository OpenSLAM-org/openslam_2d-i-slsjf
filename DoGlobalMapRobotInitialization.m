%
% function: use robot starting point to initialize new robot location
%
% input:
%   newrobot: new robot measurements  [index, x_hat, y_hat, phi_hat]
%
%
% output:
% date: 12.9, zhan % Shoudong 2006.03.15

function DoGlobalMapRobotInitialization(newrobot, robot)

global Params;
global Est;

%disp(' *** entering DoGlobalMapRobotInitialization');


x3_hat = newrobot(1,2);    % coordinate of the beacon in robot coordinate system
y3_hat = newrobot(1,3);
phi_hat = newrobot(1,4);
xr = robot(1);
yr = robot(2);
phir = robot(3);

% compute the new beacon position in global coordinate system

x3 = xr + x3_hat*cos(phir) - y3_hat*sin(phir);
y3 = yr + y3_hat*cos(phir) + x3_hat*sin(phir);
phi3 = wrap(phir + phi_hat);

%pause

index_robot = [newrobot(1,1), -Params.IndexSubmap, 0, -Params.IndexSubmap];
% initialized state vector
Est.StGlobalini = [Est.StGlobalini; index_robot, x3; index_robot, y3; index_robot, phi3];
% the current state vector just after initialization
Est.StGlobal = [Est.StGlobal; index_robot, x3; index_robot, y3; index_robot, phi3];

% adding zeros to information vector, information matrix and Cholesky
% factorization
Est.InfoVectorGlobal = [Est.InfoVectorGlobal; index_robot, 0; index_robot, 0; index_robot, 0];
inf_size=size(Est.InfoMatrixGlobal,1);
%pause
Est.InfoMatrixGlobal(inf_size+3,inf_size+3)=0;
Est.InfoMatrixCholeskyFactor(inf_size+3,inf_size+3)=0;

%     % test whether the Cholesky factorization is correct
%     test=max(max(abs(Est.InfoMatrixCholeskyFactor*Est.InfoMatrixCholeskyFactor'-Est.InfoMatrixGlobal)))
%     pause

%%%%%%%%%%%%%
%% compute small information matrix if using AMD reordering
%%%%%%%%%%%%%%%
if (Params.ReorderAMD > 0)
    state_global=Est.StGlobal;
    size_info_matrix_small=size(Est.InfoMatrixGlobalSmall,1);
    Est.InfoMatrixGlobalSmall(size_info_matrix_small+1,size_info_matrix_small+1)=100;
    Est.StGlobalSmall = [Est.StGlobalSmall; Est.StGlobal(inf_size+1,1:4)];
end
%%%%%%%%%%%%%
%% end compute small information matrix if using AMD reordering
%%%%%%%%%%%%%%%



return;

