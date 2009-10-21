%
% function: draw the uncertainty ellipses in the global map
% date: 12.13, zhan % Shoudong 2009. June -- only draw the true positions
% of beacons observed

function DoDrawBeaconEllipseGlobalMap

global Est;
global Params;
global Truth

%% get the true beacon positions

if (Params.Simulation == 1) % simulation 1
    load 535_data_5_local_maps/store_beaconsTrue
        Truth.Beacons = store_beaconsTrue;
elseif (Params.Simulation == 2) % simulation 2
    load 8240_data_50_local_maps/store_beaconsTrue
        Truth.Beacons = store_beaconsTrue;
elseif (Params.Simulation == 3) % simulation 3
    load 35188_data_700_local_maps/store_beaconsTrue
        Truth.Beacons = store_beaconsTrue;
elseif (Params.Simulation == 0) % your own local map with ground truth
    load Test_your_local_maps_with_ground_truth/store_beaconsTrue   
    Truth.Beacons = store_beaconsTrue;
end

%disp(' *** entering DoDrawBeaconEllipseGlobalMap');

%% draw the map (and uncertainty ellipse)

i=1;

while i < size(Est.StGlobal,1)
    uncer_x = Est.StGlobal(i,5);
    uncer_y = Est.StGlobal(i+1,5);

    if (Params.RecoverDiagCov==1)
        uncer_p = full(Est.PGlobalDiag(i:i+1, i:i+1));    % 2008.1.22, only recover the diagonal matrix
        CV=GetCov(uncer_p,uncer_x,uncer_y);  %
    end

    if Est.StGlobal(i,1) > 0 % feature
        if(Params.Simulation >= 0) && ( Params.Simulation<=3)
            % find the beacon index and draw the true beacon position
            beacon_index = Est.StGlobal(i,4);
            ki = find(Truth.Beacons(:,1)==beacon_index);
            plot(Truth.Beacons(ki,2),Truth.Beacons(ki,3),'b.');
        end
        if (Params.RecoverDiagCov)
            plot(CV(1,:),CV(2,:), '-r'); % draw the ellispse
        end
        plot(uncer_x,uncer_y, '.r'); % draw the estimated value 

        i=i+2;

    else % robot pose
        if (Params.RecoverDiagCov==1)
            plot(CV(1,:),CV(2,:), '-k'); % draw the ellispse
        end
        plot(uncer_x,uncer_y, '.k'); % draw the estimated value
        i=i+3;
    end


end

%drawnow;

return;

