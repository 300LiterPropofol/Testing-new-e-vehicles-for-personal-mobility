clc
clear
close all

% X means not available

% lidar_filename = 'Lidar-e-scooter-Ashwij.bag';
% bike_sensors_filename = 'e-scooter-Ashwij.bag';
% lidar_filename = 'Lidar-e-scooter-Estelle.bag';
% bike_sensors_filename = 'e-scooter-Estelle.bag';
% lidar_filename = 'Lidar-e-scooter-Marco.bag'; X
% bike_sensors_filename = 'e-scooter-Marco.bag'; X
% lidar_filename = 'Lidar-e-scooter-Nithin.bag';
% bike_sensors_filename = 'e-scooter-Nithin.bag';
% lidar_filename = 'Lidar-e-scooter-Weicheng.bag'; X
% bike_sensors_filename = 'e-scooter-Weicheng.bag'; X
lidar_filename = 'Lidar-e-scooter-Alex.bag';
bike_sensors_filename = 'e-scooter-Alex.bag';
% lidar_filename = 'Lidar-e-scooter-Di.bag'; X
% bike_sensors_filename = 'e-scooter-Di.bag'; X
%%
run ReadIn_lidar_and_bike_sensors.m
%%
run Save_summarized_datatable.m
%%
run Split_and_save_tasks.m
%%
% you need to type the task id here, 
%which number to type in you need to manually look at the test plan, 
%see which task we are doing in those two files above
run cut_out_test_slot.m
%%
run Segmentation_and_PI_compute.m
