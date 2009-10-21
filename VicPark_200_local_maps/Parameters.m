%
% function: set initial parameters for SLAM -- for Victoria park data set
%
% 
%
%
% date: 12.9, zhan

function Parameters

global Params;

randn('state',0)
format short

% noise estimation 
Params.sigv = 0.02;                    
Params.sigomega = 2*pi/180;            
Params.sigr = 0.01;                    
Params.sigtheta = (1*pi/180);        
Params.sigrAdd = 0.1;                      % additional noise \omega_s in Jose Guivant's paper (tree extraction)

Params.Q = [Params.sigv*Params.sigv 0; 0 Params.sigomega*Params.sigomega];

% add stabilizing noise all the time --  2006.12.1, Shoudong 

Params.QStab = [0.2^2, 0, 0; 0, 0.2^2, 0; 0, 0, (pi/180)^2]; 


Params.a = 3.78;            % 2005.7.20, from Ute Modelling Information. online doc for VP data set
Params.b = 0.5;
Params.L = 2.83;
Params.H = 0.76;
                                
                                
return;
