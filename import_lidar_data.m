function [ data ] = import_lidar_data( file )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

fprintf('Importing %s...\n', file)

lidar_fov_deg = 190;

raw = csvread(file);

data.Timestamp_s = raw(:, 1);
data.Angle_rad = deg2rad(linspace(-lidar_fov_deg/2, lidar_fov_deg/2, 1521));
data.Distance_m = raw(:, 2:end);

end

