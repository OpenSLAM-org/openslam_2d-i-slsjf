%
% 12.3, zhan % simplified by Shoudong 2006.03.18

function DoFigure

global Params;

% draw the global map, use Est.StGlobal, Est.PGlobal
figure(3);

hold on
xlabel('X(m)');
ylabel('Y(m)');
%axis square

if(Params.IndexSubmapStart == 1)

if (Params.Simulation == 1) % 535 steps data
    axis ([-10, 40, -10, 40])         % for 535 steps data
elseif (Params.Simulation == 2) % 8240 steps data
    axis ([-10, 160, -10, 160])         % for 8240 steps data
elseif (Params.Simulation == 3) % 35188 steps data
    axis ([-10, 300, -10, 300])         % for 35188 steps
elseif (Params.Simulation == 4)||(Params.Simulation == 7) % DLR data
    axis ([-50, 20, -35, 35])         % for DLR dataset
elseif (Params.Simulation == 5)||(Params.Simulation == 8) %victoria park
    axis ([-150, 250, -100, 300])         % for victoria park
end

else
    axis equal
%        axis ([-100, 100, -100, 100])  % or define the axis yourself here
end

grid on

DoDrawBeaconEllipseGlobalMap;

zoom on
%
% print -depsc 'MapGlobal.eps'

return;

