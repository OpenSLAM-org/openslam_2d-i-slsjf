This folder includes:

(1) 200 local maps -- localmap_i.mat contains state vector estimate localmap_st and covariance matrix localmap_P

the first column of localmap_st is the feature index (first three rows index=0 means robot), 
the second column of localmap_st is the estimate value

(2) VicPark_clean_data.mat -- the preprocessed Victoria Park odometry and observation data

format: first column -- odometry (1) or observation (0)

each row of odometry: [1(odometry) index_start_pose index_end_pose dx dy dphi Pxx Pxy Pxphi Pyy Pyphi Pphiphi]

each row of observation: [0(observation) index_obs_pose feature_index dx dy Pxx Pxy Pyy 0 0 0 0]

Using this data, there is no need to know the vehicle model and the sensor position (the format is similar to that of DLR dataset)
============================

(3) ControlData.mat -- control data, 

format: [time speed turnangle]

(4) data_sensor_clean_clean.mat -- observation data, 

format: [time range bearing x_est y_est feature_index] 
(feature_index=-1 means the feature is a poor quality feature, the obervation should be ignored)

(5) Parameters.m --- parameters used in SLAM using Victoria park data

(6) DoPredictionOptimized.m -- the prediction function used for Victoria park data (which contains the vehicle model)

(7) DoUpdate.m --- the update function used for Victoria park data (which contains the observation model)
