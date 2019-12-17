% this code need to be run after ReadIn_lidar_and_bike_sensors.m has been
% run so that the all relevant table is already in workspace now
% this code aims to save the intermediate summarized data table, which gives a
% neat form of reference if any more data is needed in the future

% the data that need to be saved are:
% TimeStamp_lidar
% cartersian_coordinate
% cartersian_coordinate_interested
% SignalTable_adc
% SignalTable_imu
% SignalTable_ADCIMU_synchronized
% SignalTable_after_preprocessing

save('intermediate_datatable_backup.mat','SignalTable_adc','SignalTable_imu','SignalTable_ADCIMU_synchronized','SignalTable_after_preprocessing');
disp('Good! intermediate data has been saved for back up');