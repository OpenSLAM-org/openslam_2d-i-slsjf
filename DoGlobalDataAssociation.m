%
% function: data association between local map and global map
%
% input: globalmap, local map
%
% output: match_matrix
%
% last modified: 24/09/2009, Shoudong

function DoGlobalDataAssociation

global Params;
global Match;
global Est;

% disp(' *** entering DoGlobalDataAssociation');

% to record the time used for global state recovery
timeStart = cputime;

% use the joint_match_beacons_NN code to find the match
% use the state estimation in the global map and the local map
%
%% local map as the observation

localmap_st = Est.St;

localmap_P = Est.P;

n2 = size(localmap_st,1);

obs = localmap_st(4:n2,4);

obscov = localmap_P(4:n2,4:n2);

num_obs_beacon = size(obs,1)/2;

% global map are beacons
globalmap_st = Est.StGlobalSelected;

globalmap_P = Est.PGlobalSelected;

num = size(globalmap_st,1);

if num==3 %% no potentially matched beacons, all the observed beacons are new beacons

    match_local_map = zeros(num_obs_beacon,4);

    for i=1:num_obs_beacon
        match_local_map(i,:)=[Params.IndexSubmap, i, -100, localmap_st(2*i+2,3)];
    end

else %% at least one potentially matched beacons

    % change the order of robot and beacons -- put robot in front
    globalmap_st = [globalmap_st(num-2:num,:);globalmap_st(1:num-3,:)];

    globalmap_P = [globalmap_P(num-2:num,:);globalmap_P(1:num-3,:)];
    globalmap_P = [globalmap_P(:,num-2:num),globalmap_P(:,1:num-3)];

    %pause
    % transfer the global beacons to the relative beacons
    [beac,beaccov] = Translocalmaprobot(globalmap_st(:,5),globalmap_P);

    %% call the nearest neighbout data association
    correspondance = joint_match_beacons_NN(beac,beaccov,obs,obscov);

    %pause
    % change the correspondance to the global map index
    newcorrespondance=correspondance;
    %select = Est.SelectedBeaconForDataAssociation;
    for j=1:size(correspondance,1)
        if correspondance(j,2)>0
            newcorrespondance(j,2) = Est.SelectedBeaconForDataAssociation(2*correspondance(j,2)-1,2);
        end
    end

    match_index = [newcorrespondance,-100*ones((n2-3)/2,1),-100*ones((n2-3)/2,1)];

    for i=1:size(correspondance,1)
        match_index(i,4) = localmap_st(2*i+2,3);
        if correspondance(i,2) > 0
            match_index(i,3) = globalmap_st(2*correspondance(i,2)+3,4);
            %% regard poor observation as wrong data association
        elseif correspondance(i,2) == -1
            match_index(i,3) = -1;
        end
    end

    %  match_index
    %  pause

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % check whether the match is correct (when feature ID is available)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    if (Params.Simulation > 0)
        oldbeacon=zeros(num_obs_beacon,2);

        for i = 4:2:size(Est.St,1)  % robot is in it, so start from 4
            oldbeacon_sign = 0;
            for j = 1:size(Est.StGlobal,1)
                if(Est.St(i,3)==Est.StGlobal(j,4))
                    % old beacon!
                    %oldbeacon = [oldbeacon; Data.RangeBearing(i,:)];
                    oldbeacon_sign = 1;
                    oldbeacon((i-2)/2,:) = [(i-2)/2 Est.St(i,3)];
                    break;
                end
            end
            if(oldbeacon_sign == 0)
                oldbeacon((i-2)/2,:) = [(i-2)/2 -100];
            end
        end

        % oldbeacon
        match_index_2_column = [match_index(:,1),match_index(:,3)];

        match_diff = max(max(abs(oldbeacon-match_index_2_column)));



        if match_diff>0.01 % wrong GlobalDataAssociation

            oldbeacon
            match_index_2_column
            Est.WrongDataAssocNum = [Est.WrongDataAssocNum; Params.IndexSubmap];
            diff_match=oldbeacon;
            diff_match(:,2)=oldbeacon(:,2)-match_index_2_column(:,2);
            diff_match
            disp(' *** wrong GlobalDataAssociation');
            index_submap = Params.IndexSubmap

            % for debug: show wrong data association in figure

            if (Params.ShowWrongDataAssociation == 1)
                pause

                localmap_st
                globalmap_st
                pause

                save DA_wrong beac beaccov obs obscov

                %%%% for debug: compare the transfered beacons and local map
                if(1)

                    globalmap_trans = [globalmap_st(4:size(globalmap_st,1),1), globalmap_st(4:size(globalmap_st,1),4), beac];
                    localmap_st_beac = localmap_st(4:n2,2:4);

                    %%% draw the figure

                    figure(100)
                    hold on

                    St=localmap_st_beac(:,2:3);

                    num_beac=size(St,1)/2;

                    P=obscov;

                    for i = 1:num_beac
                        uncer_p = P(i*2-1:i*2, i*2-1:i*2);      
                        uncer_x = St(i*2-1,2);
                        uncer_y = St(i*2,2);
                        CV=GetCov(uncer_p,uncer_x,uncer_y);  
                        plot(CV(1,:),CV(2,:),'-r');
                        plot(uncer_x,uncer_y,'.r','linewidth',2);
                        text(uncer_x+0.03,uncer_y+0.03,num2str(St(i*2,1)));
                        hold on
                        %  pause
                    end
                    %%%
                    St=globalmap_trans(:,2:3);

                    num_beac=size(St,1)/2;

                    P=beaccov;

                    for i = 1:num_beac
                        uncer_p = P(i*2-1:i*2, i*2-1:i*2);      
                        uncer_x = St(i*2-1,2);
                        uncer_y = St(i*2,2);
                        CV=GetCov(uncer_p,uncer_x,uncer_y);  
                        plot(CV(1,:),CV(2,:),'-b');
                        plot(uncer_x,uncer_y,'+b','linewidth',2);
                        text(uncer_x+0.03,uncer_y+0.03,num2str(St(i*2,1)));
                        hold on
                        %pause
                    end

                    pause
                    clf
                end
                %%% end draw the figure
            end
            %%%% end for debug

        end
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % end check whether the match is correct
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end

    match_local_map = [Params.IndexSubmap*ones(size(match_index,1),1), match_index(:,1:2),match_index(:,4)];


end

% add the robot in the match matrix
match_local_map = [match_local_map; Params.IndexSubmap, 0, -Params.IndexSubmap, -Params.IndexSubmap];
%pause

%match_local_map=match_local_map

Match.LocalMap = match_local_map;

% format: [submap number, index_in_local_map, index_global_map, ID]

%pause

%% to record the time used in Data association
timeUsed = cputime - timeStart;
Est.timeUsedDataAssoc = [Est.timeUsedDataAssoc;  timeUsed];

return;

