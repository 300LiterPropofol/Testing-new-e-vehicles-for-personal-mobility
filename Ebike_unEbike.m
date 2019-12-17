clc
clear
close all
%%
% lidar_filename = 'Lidar-convbike-Ashwij.bag'; % manual input
% bike_sensors_filename = 'convbike-Ashwij.bag'; % manual input
% lidar_filename = 'Lidar-convbike-Estelle.bag'; % manual input
% bike_sensors_filename = 'convbike-Estelle.bag'; % manual input
% lidar_filename = 'Lidar-convbike-Marco.bag'; % manual input
% bike_sensors_filename = 'convbike-Marco.bag'; % manual input
% lidar_filename = 'Lidar-convbike-Nithin.bag'; % manual input
% bike_sensors_filename = 'convbike-Nithin.bag'; % manual input
% lidar_filename = 'Lidar-convbike-Weicheng.bag'; % manual input
% bike_sensors_filename = 'convbike-Weicheng.bag'; % manual input
% lidar_filename = 'Lidar-convbike-Alex.bag'; % manual input
% bike_sensors_filename = 'convbike-Alex.bag'; % manual input
% lidar_filename = 'Lidar-convbike-Di.bag'; % manual input
% bike_sensors_filename = 'convbike-Di.bag'; % manual input

% lidar_filename = 'Lidar-convbike-Nithin2.bag'; % manual input
% bike_sensors_filename = 'convbike-Nithin2.bag'; % manual input
%%
run ReadIn_lidar_and_bike_sensors.m
%%
run Save_summarized_datatable.m
%%
run Split_and_save_tasks.m

run cut_out_test_slot.m
%%
run Segmentation_and_PI_compute.m
