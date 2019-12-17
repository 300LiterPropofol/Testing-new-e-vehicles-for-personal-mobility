clc
clear
close all

% X means not available

% lidar_filename = 'Lidar-segway-Ashwij.bag'; % manual input X
% bike_sensors_filename = 'segway-Ashwij.bag'; % manual input X
% lidar_filename = 'Lidar-segway-Estelle.bag'; % manual input X
% bike_sensors_filename = 'segway-Estelle.bag'; % manual input X
% lidar_filename = 'Lidar-segway-Marco.bag'; % manual input X
% bike_sensors_filename = 'segway-Marco.bag'; % manual input X
% lidar_filename = 'Lidar-segway-Nithin.bag'; % manual input
% bike_sensors_filename = 'segway-Nithin.bag'; % manual input
% lidar_filename = 'Lidar-segway-Weicheng.bag'; % manual input
% bike_sensors_filename = 'segway-Weicheng.bag'; % manual input
% lidar_filename = 'Lidar-segway-Alex.bag'; % manual input
% bike_sensors_filename = 'segway-Alex.bag'; % manual input
lidar_filename = 'Lidar-segway-Di.bag'; % manual input
bike_sensors_filename = 'segway-Di.bag'; % manual input

% lidar_filename = 'Lidar-skateboard-Estelle.bag'; % manual input X
% bike_sensors_filename = 'skateboard-Estelle.bag'; % manual input X
% lidar_filename = 'Lidar-skateboard-Marco.bag'; % manual input X
% bike_sensors_filename = 'skateboard-Marco.bag'; % manual input X



run ReadIn_lidar_and_bike_sensors.m
run Save_summarized_datatable.m
run Split_and_save_tasks.m


% you need to type the task id here, 
%which number to type in you need to manually look at the test plan, 
%see which task we are doing in those two files above

run cut_out_test_slot.m
%%
run Segmentation_and_PI_compute.m
