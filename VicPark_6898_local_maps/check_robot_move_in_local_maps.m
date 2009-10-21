% check robot move 
not_move_index = [];
num_localmaps = 6898

StabNoise = 1e-8;

for i=1:num_localmaps
    index = num2str(i)
    filename = strcat('localmap_', index);

    load(filename);

    %localmap_st

    x = localmap_st(1,2);

    y = localmap_st(2,2);

    phi = localmap_st(3,2);

    if x==0 && y==0 && phi==0
        disp('robot did not move')
        
        not_move_index = [not_move_index; i];
        pause
    end
     %% if robot did not move in the local map, increase the 0 uncertainty a bit
    if (localmap_P(1,1)<StabNoise)||(localmap_P(2,2)<StabNoise)||(localmap_P(3,3)<StabNoise)
        % add a bit on P to make it non-singular
        localmap_P(1:3,1:3) = localmap_P(1:3,1:3) + StabNoise*[1, 0, 0; 0, 1, 0; 0, 0, 1];
         
     disp('localmap_P too small')
         pause
    end
end

not_move_index

size(not_move_index)