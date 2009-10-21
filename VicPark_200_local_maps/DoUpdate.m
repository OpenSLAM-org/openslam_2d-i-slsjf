% function: update step in EKF SLAM 

function DoUpdate

global Params;
global AidSlam;


a = Params.a;
b = Params.b;

numOldBeacon = size(AidSlam.OldBeacons,1);

% here, use sequencial method: each time, only use one measurement to update all. this is easy for program
% use EKF to update
for i=1:numOldBeacon
    % copy from DoCIAllMeasureUpdateEIF
    % get robot location
    xc = AidSlam.St(1,2);
    yc = AidSlam.St(2,2);
    phi = AidSlam.St(3,2);

    xv = xc + a*cos(phi) - b*sin(phi);
    yv = yc + a*sin(phi) + b*cos(phi);


    % the meaning of value returned (index in AidSlam.St)

    kk = find(AidSlam.St(:,1)==AidSlam.OldBeacons(i,6));

    if (size(kk,1)<1)|(size(kk,1)>2)
        disp(' *** something wrong -- not an old beacon');
    else
        indexInAidSt = kk(1);
    end


    % here, should use the estimate of the beacon in Est.RobotBeaconEst1, coz this is to estimate Est.RobotBeaconEst1
    xb = AidSlam.St(indexInAidSt,2);
    yb = AidSlam.St(indexInAidSt+1,2);

    % get measurement
    zReal = [AidSlam.OldBeacons(i,2); AidSlam.OldBeacons(i,3)];       % this is the right one
    zReal(2,1) = wrap(zReal(2,1));

    % get Jacobian of observation model
    [m,n] = size(AidSlam.St);
    jh = zeros(2,m);
    index = indexInAidSt;

    square_sum = (  (xv-xb)^2 + (yv-yb)^2  );
    square_root = square_sum^0.5;

    jh(1,1) = (xv-xb)/square_root;  % partial r; partial x_c
    jh(1,2) = (yv-yb)/square_root;  % partial r; partial y_c
    tmp = ( (xv-xb)*(-a*sin(phi)-b*cos(phi))  +  (yv-yb)*(a*cos(phi)-b*sin(phi))  );
    jh(1,3) = tmp/square_root;                    % partial r; partial phi
    jh(1,index) = (-1)*(xv-xb)/square_root;     % partial r; partial x_i
    jh(1,index+1) = (-1)*(yv-yb)/square_root;   % partial r; partial y_i

    jh(2,1) = (yb-yv)/square_sum;               % partial theta; partial x_c
    jh(2,2) = (-1)*(xb-xv)/square_sum;          % partial theta; partial y_c
    tmp2 = -(a*cos(phi)-b*sin(phi))*(xb-xv) - (yb-yv)*(a*sin(phi)+b*cos(phi));
    jh(2,3) = -1 + tmp2/square_sum;                               % partial theta; partial phi
    jh(2,index) = (-1)*(yb-yv)/square_sum;      % partial theta; partial x_i
    jh(2,index+1) = (xb-xv)/square_sum;         % partial theta; partial y_i


    % predicted measurement
    rangek = ((yb - yv)^2 + (xb - xv)^2)^(1/2);
    thetak = atan2((yb - yv),(xb - xv)) - phi + pi/2 ;
    zPredicted = [rangek; thetak];
    zPredicted(2,1) = wrap(zPredicted(2,1));

    % innovation and its cov
    
                % noise on the range to tree centre
    R = [(Params.sigrAdd+AidSlam.OldBeacons(i,2)*Params.sigr)^2 0; 0 Params.sigtheta^2];    % 2007.09.23, Lina
    
    innovationCov = jh*AidSlam.P*jh' + R;
    innovation = (zReal - zPredicted);
    innovation(2,1) = wrap(innovation(2,1));


    % update
    w = AidSlam.P*jh'*inv(innovationCov);
    AidSlam.St(:,2) = AidSlam.St(:,2) + w*(innovation);

    %     state = AidSlam.St
    %     pause
    AidSlam.P = AidSlam.P - w*innovationCov*w';

end

return;