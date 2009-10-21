% function: prediction step in EKF SLAM -- for Victoria part data set


% Shoudong 2006.11.30


function DoPredictionOptimized(dt)

global Params;
global Store;
global Figure;
global Data;
global Beacon;
global Truth;
global Est;
global AidSlam;


L = Params.L;
H = Params.H;


xk = AidSlam.St(1,2);
yk = AidSlam.St(2,2);
phik = AidSlam.St(3,2);

% disabled on 2005.7.24. 
% dt = (Data.CurrentTime - Data.LastTime)/1000;  % coz unit is milli-sec

ve = Data.Speed;
gamk = wrap(Data.Turnangle);
vc = ve/(1-tan(gamk)*H/L);      

stkp1(1,1) = xk + dt*vc*cos(phik);   % original
stkp1(2,1) = yk + dt*vc*sin(phik);           % original
stkp1(3,1) = phik + dt*vc*tan(gamk)/L; % original


m = size(AidSlam.P,1);
f = eye(3,3); % for vehicle only, Shoudong 
f(1,1) = 1;
f(1,2) = 0;
f(1,3) = -dt*vc*sin(phik);
f(2,1) = 0;
f(2,2) = 1;
f(2,3) = dt*vc*cos(phik);
f(3,1) = 0;
f(3,2) = 0;
f(3,3) = 1;



g = zeros(3,2); % for vehicle only, Shoudong 
g(1,1) = dt*cos(phik);
g(1,2) = 0;                           % original 
g(2,1) = dt*sin(phik);
g(2,2) = 0;                           % original
g(3,1) = dt*tan(gamk)/L;
g(3,2) = dt*vc/(cos(gamk)^2*L);

% Params.Q is about [ve, gamk]'
jG = zeros(2,2);
jG(1,1) = 1/(1-tan(gamk)*H/L);
jG(1,2) = (ve*H)/(cos(gamk)^2*  (1-tan(gamk)*H/L)^2  *L);
jG(2,1) = 0;
jG(2,2) = 1;

newControlNoise = jG*Params.Q*jG';

pkp1(1:3,1:3) = f*AidSlam.P(1:3,1:3)*f' + g*newControlNoise*g' ;   % original

pkp1(1:3,1:3) = pkp1(1:3,1:3) + Params.QStab*dt; % stabilizing noise, Shoudong 2006 Nov 13

pkp1(4:m,1:3) = AidSlam.P(4:m,1:3)*f';

pkp1(1:3,4:m) = f*AidSlam.P(1:3,4:m);


AidSlam.St(1:3,2) = stkp1;
AidSlam.P(1:3,:) = pkp1(1:3,:);
AidSlam.P(4:m,1:3) = pkp1(4:m,1:3);

return;