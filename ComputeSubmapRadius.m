% compute the radius of submap -- distance from robot to the farthest
% beacon plus the uncertainty
%
% for find possible matched beasons in data association
%
% Shoudong, 2006, July

function radius = ComputeSubmapRadius(localmap_st,localmap_P)

global Params;

state = localmap_st(:,4);
P = localmap_P;

num_beac = (size(state,1)-3)/2;

radius = 0;

for i=1:num_beac
    distance = sqrt((state(2*i+2)-state(1))^2+(state(2*i+3)-state(2))^2);
    % uncertainty of the beacon
    cov = P(2*i+2:2*i+3,2*i+2:2*i+3);
    % times the uncertainty with multiplier (e.g. 3 sigma)
    uncertainty = max(eig(sqrtm(cov)))*Params.SigmaMultiplier;

    %     if uncertainty>1 % check whether the uncertainty is too large
    %         uncertainty
    %        % pause
    %     end
    distance = distance+uncertainty;
    if distance>radius % find the largest distance
        radius = distance;
    end
end


return;
