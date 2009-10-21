%
% function: draw all beacon uncertainty ellipse
%
% date: 12.13, zhan

load localmap_2938
%load EKF_SLAM_2008_Oct_29



St=localmap_st;

num_beac=(size(St,1)-3)/2

P=localmap_P;


figure(1)
%axis ([-1 11, -6 6])
%axis square
hold on

for i = 1:(size(St,1)-3)/2
    uncer_p = P(i*2+2:i*2+3, i*2+2:i*2+3);        % enlarge it to make it clear
    uncer_x = St(i*2+2,2);
    uncer_y = St(i*2+3,2);
    CV=GetCov(uncer_p,uncer_x,uncer_y);  % by wangzhan, make it large on purpose, not now
    plot(CV(1,:),CV(2,:),'-r');
    plot(uncer_x,uncer_y,'.b','linewidth',2);
     text(uncer_x+0.03,uncer_y+0.03,num2str(St(i*2+2,1)));
    hold on
    %pause
end

%%% draw robot uncertainty
uncer_p = P(1:2, 1:2);        % enlarge it to make it clear
uncer_x = St(1,2);
uncer_y = St(2,2);
CV=GetCov(uncer_p,uncer_x,uncer_y);  % by wangzhan, make it large on purpose, not now

plot(CV(1,:),CV(2,:),'-r');
hold on

%%%%% draw robot
plot(uncer_x,uncer_y,'>b','linewidth',4);

hold on

% end of drawing uncertainty ellipse

%legend('range-bearing local map', 'bearing-only local map');

ylabel('Y(m)');
xlabel('X(m)');

%axis equal
%axis auto
zoom on

return;

