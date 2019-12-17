% this code should run after cut_out_test_slot.m has been run
% this code aims to do evaluate/analyze the task that we read in this code

%In the following step we are going to do segmentation,
%segmentation are Acceleration behaviour(in short Acc), Deceleration
%behaviour (in short Dec), Constantly riding at 15 km/h behaviour(in short
%const15), Constantly riding at 7 km/h behaviour (in short const7) ,slalom
%behaviour (in short slalom)

% before we start segmentation, here we recap the test protocol somehow to
% give an impression about what the data should be like. We have 3 tasks,
% task 1: straight course, accelerate slow/ very safely to 15km/h, keep
% speed at 15km/h, and then brake slow/very safely
% task 2: straight course, accelerate fast to 15km/h, keep speed at 15km/h,
% and then brake fast
% task 3: slalom, accelerate to 7km/h, slalom go through the cones.

% we have alraedy distinguish the task id in cut_out_test_slot.m, task_id=1/2/3

clear
close all

%%
load testslot_second_split.mat
how_many_tasks = length(test_slot_cell);
segmentation_and_PI_cell = cell(how_many_tasks,1); % initialization
for i = 1:how_many_tasks
    % first of all plot all relevant data that we want to manually check
    % plot the trajectory of our participant
    figure;
    scatter(test_slot_cell{i}.trajectory_reconstruction(:,1),test_slot_cell{i}.trajectory_reconstruction(:,2),'b');
    xlabel('unit (m)');
    ylabel('unit (m)');
    title('trajectory reconstruction from lidar data');
    axis equal
    
    % plot animation for trajectory construction, to directly see how the
    % detected object point goes
    current_slot_frames_number = length(test_slot_cell{i}.trajectory_reconstruction(:,1));
    current_slot_frames = test_slot_cell{i}.trajectory_reconstruction;
    current_slot_unselected_points = test_slot_cell{i}.unselected_clusters_record; % in current_slot_frames_number x 1 cell form
    fig=figure;
    %aviobj = VideoWriter(['C:\MasterinCTH\TME180Project\Signal procressing and data analysis\E-scooter analysis\E-scooter figure outcome\Alex\' datestr(now,30) '.avi']);
    %open(aviobj);
    for k = 1:current_slot_frames_number
	plot(current_slot_frames(k,1),current_slot_frames(k,2),'b.')
    hold on
    if ~isempty(current_slot_unselected_points{k})
    scatter(current_slot_unselected_points{k}(:,1),current_slot_unselected_points{k}(:,2));
    end
    hold on
	axis([-28 37 0 7])
    xlabel('unit (m)');
    ylabel('unit (m)');
    title('trajectory reconstruction from lidar data');
	M(k) = getframe;
    % Get it as an avi-frame
    %F = getframe(fig);
    % Add the frame to the avi
    %writeVideo(aviobj,F);  
    end
    %close(aviobj);
    
    % plot speed data from lidar
    figure;
    x_speed = test_slot_cell{i}.x_y_velocity_from_lidar(:,1);
    y_speed = test_slot_cell{i}.x_y_velocity_from_lidar(:,2);
    Speed_from_lidar = sqrt(x_speed.^2 + y_speed.^2);
    Speed_from_lidar = smoothdata(Speed_from_lidar,'Gaussian',10);
    plot(test_slot_cell{i}.TimeStamp_lidar,Speed_from_lidar);
    xlabel('TimeStamp_lidar (s)');
    ylabel('Speed (m/s)');
    title('Speed derivated from lidar data');
    
    % plot acc-x data
    figure;
    plot(test_slot_cell{i}.TimeStamp_imu,test_slot_cell{i}.LinearAcceleration_x_filtered);
    xlabel('TimeStamp_imu (s)');
    ylabel('unit (m/s^2)');
    title('LinearAcceleration x check');
    
    % plot gyro-x data
    figure;
    plot(test_slot_cell{i}.TimeStamp_imu,test_slot_cell{i}.AngularVelocity_x_filtered);
    xlabel('TimeStamp_imu (s)');
    ylabel('unit (rad/s)');
    title('AngularVelocity x check');
    
    % plot steering angle
    figure;
    plot(test_slot_cell{i}.TimeStamp_imu,test_slot_cell{i}.SteeringAngle_filtered);
    xlabel('TimeStamp_imu (s)');
    ylabel('unit (deg)');
    title('Steering Angle check');
    
    % plot steering rate
    figure;
    plot(test_slot_cell{i}.TimeStamp_imu,test_slot_cell{i}.steering_rate);
    xlabel('TimeStamp_imu (s)');
    ylabel('unit (deg/s)');
    title('Steering rate check');
end   
%--------------------------------------------------------------------------------------  
for i = 1:how_many_tasks
    x_speed = test_slot_cell{i}.x_y_velocity_from_lidar(:,1);
    y_speed = test_slot_cell{i}.x_y_velocity_from_lidar(:,2);
    Speed_from_lidar = sqrt(x_speed.^2 + y_speed.^2);
    Speed_from_lidar = smoothdata(Speed_from_lidar,'Gaussian',10);
    Speed_from_lidar_synchronized = interp1(test_slot_cell{i}.TimeStamp_lidar-test_slot_cell{i}.TimeStamp_lidar(1),Speed_from_lidar,test_slot_cell{i}.TimeStamp_imu-test_slot_cell{i}.TimeStamp_imu(1),'linear','extrap');   
    if test_slot_cell{i}.task_id == 1 || test_slot_cell{i}.task_id == 2
% when task_id=1/2, use function getAccIndices
% function [start_acc_index, const_speed_start, end_brake_index] = getAccIndices(time, speed, min_speed_thsd, req_speed, min_t_over_thsd)
% time = TimeStamp_imu
% speed = Speed_filtered
min_speed_threshold = 1; % unit m/s !!!!!!!will be modified later
request_speed = 15/3.6; % unit m/s !!!!!!!will be modified later
min_t_over_threshold = 8; % unit s !!!!!!will be modified later
[start_acc_index, const_speed_start, end_brake_index] = getAccIndices(test_slot_cell{i}.TimeStamp_imu, Speed_from_lidar_synchronized, min_speed_threshold, request_speed, min_t_over_threshold);
% continue working on task_id=1/2, use function getBrakeIndex
% [dec_start_index, brake_start_index] = getBrakeIndex(end_brake_index, speed_thsd, time, acc_x, speed, brake_t_over_thsd)
% dec_start_index is earlier than brake_start_index?
% Acc is from start_acc_index to const_speed_start
% const15 is from const_speed_start to dec_start_index
% Dec is from brake_start_index to end_brake_index
speed_threshold = 15/3.6; % unit m/s !!!!!will be modified later
brake_t_over_threshold = 8; % init s, !!!!!!will be mdified later
acc_x = test_slot_cell{i}.LinearAcceleration_x_filtered;
[dec_start_index, brake_start_index] = getBrakeIndex(end_brake_index, speed_threshold, test_slot_cell{i}.TimeStamp_imu, acc_x, Speed_from_lidar_synchronized, brake_t_over_threshold);

% recap again
% Acc is from start_acc_index to const_speed_start
% const15 is from const_speed_start to dec_start_index
% Dec is from brake_start_index to end_brake_index

% segmentation check, if the segmentation is unreasonable, it doesn't make
% sense to continue further analysis
if (end_brake_index>brake_start_index && brake_start_index>=dec_start_index &&dec_start_index>const_speed_start &&const_speed_start>start_acc_index)~=1
    error('segmentation is wrong, please check the code again');
end

% until now, we have Acc, Dec, Const15 segmentation, according to paper Eval of a new narrow and tiltable electric cycle (e-trike) concept - final report
% we need to firstly extract :
% segmentation time [s] (for Acc, Dec)
% speed [m/s] (for Const15)
% steering rate [deg/s] (for Const15)
% roll rate [deg/s] (for Const15)
% steering angle [deg] (for Const15)
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.task_id = test_slot_cell{i}.task_id; 
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.Acc_timerange=test_slot_cell{i}.TimeStamp_imu(start_acc_index:const_speed_start);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.Acc_time_start=test_slot_cell{i}.TimeStamp_imu(start_acc_index);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.Acc_time_end=test_slot_cell{i}.TimeStamp_imu(const_speed_start);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.Dec_timerange=test_slot_cell{i}.TimeStamp_imu(brake_start_index:end_brake_index);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.Dec_time_start=test_slot_cell{i}.TimeStamp_imu(brake_start_index);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.Dec_time_end=test_slot_cell{i}.TimeStamp_imu(end_brake_index);

segmentation_and_PI_cell{i}.Segmentation_signal_extraction.speed_range_const15=Speed_from_lidar_synchronized(const_speed_start:dec_start_index);

segmentation_and_PI_cell{i}.Segmentation_signal_extraction.steering_rate_range_const15=test_slot_cell{i}.steering_rate(const_speed_start:dec_start_index);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.roll_rate_range_const15=test_slot_cell{i}.AngularVelocity_x_filtered(const_speed_start:dec_start_index);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.steering_angle_range_const15=test_slot_cell{i}.SteeringAngle_filtered(const_speed_start:dec_start_index);

% after extracting the signal, we need to computer performance indicator
% based on the signal we have extracted.
% time [s] (for Acc, Dec)
% mean speed [m/s] (for Const15)
% time delay between roll rate and steering rate [s] (for Const15)
% R^2 of linear fit of steering rate and roll rate [-] (for Const15)
% mean absolute roll rate [deg/s] (for Const15)
% mean absolute steering angle [deg] (for Const15)
segmentation_and_PI_cell{i}.Performance_indicator.task_id=test_slot_cell{i}.task_id;
segmentation_and_PI_cell{i}.Performance_indicator.Acc_time=test_slot_cell{i}.TimeStamp_imu(const_speed_start)-test_slot_cell{i}.TimeStamp_imu(start_acc_index);
segmentation_and_PI_cell{i}.Performance_indicator.Dec_time=test_slot_cell{i}.TimeStamp_imu(end_brake_index)-test_slot_cell{i}.TimeStamp_imu(brake_start_index);
segmentation_and_PI_cell{i}.Performance_indicator.mean_speed_const15=mean(Speed_from_lidar_synchronized(const_speed_start:dec_start_index));
% use function delaySteerGyro to compute time delay
% function value = delaySteerGyro(time, st_rate, gyro_x, segment_start, segment_end)

    time=test_slot_cell{i}.TimeStamp_imu;
    gyro_x=test_slot_cell{i}.AngularVelocity_x_filtered;
    st_rate=test_slot_cell{i}.steering_rate;

segmentation_and_PI_cell{i}.Performance_indicator.delay_between_rollrate_and_steeringrate_const15 = delaySteerGyro(time, st_rate, gyro_x, const_speed_start, dec_start_index);

% use function r2SteerGyro to compute R^2 of linear fit of steering rate and roll rate
% function value = r2SteerGyro(st_rate, gyro_x, p_rate, segment_start, segment_end)
% p_rate = polyfit(st_rate,gyro_x,1); p_rate has 2 values inside, k(slope)
% and b(interception)
p_rate = polyfit(st_rate(const_speed_start:dec_start_index),gyro_x(const_speed_start:dec_start_index),1);
segmentation_and_PI_cell{i}.Performance_indicator.R2_between_rollrate_and_steeringrate_const15 = r2SteerGyro(st_rate, gyro_x, p_rate, const_speed_start, dec_start_index);
segmentation_and_PI_cell{i}.Performance_indicator.absolute_rollrate_const15=mean(abs(test_slot_cell{i}.AngularVelocity_x_filtered(const_speed_start:dec_start_index)));
segmentation_and_PI_cell{i}.Performance_indicator.absolute_steering_angle_const15=mean(abs(test_slot_cell{i}.SteeringAngle_filtered(const_speed_start:dec_start_index)));
%--------------------------------------------------------%
    elseif test_slot_cell{i}.task_id == 3
% when task_id=3, we not only needs to do whatever we have done when task_id=1/2, i.e 
% distinguish Acc,Dec and Const15(here it changes to Const7), but also we need to detect Slalom too
% firstly , as before,use function getAccIndices
% function [start_acc_index, const_speed_start, end_brake_index] = getAccIndices(time, speed, min_speed_thsd, req_speed, min_t_over_thsd)
% time = TimeStamp_imu
% speed = Speed_filtered
min_speed_threshold = 1; % unit m/s !!!!!!!will be modified later
request_speed = 7/3.6; % unit m/s !!!!!!!will be modified later
min_t_over_threshold = 8; % unit s !!!!!!will be modified later
[start_acc_index, const_speed_start, end_brake_index] = getAccIndices(test_slot_cell{i}.TimeStamp_imu, Speed_from_lidar_synchronized, min_speed_threshold, request_speed, min_t_over_threshold);
% continue working on task_id=3, use function getBrakeIndex
% [dec_start_index, brake_start_index] = getBrakeIndex(end_brake_index, speed_thsd, time, acc_x, speed, brake_t_over_thsd)
% dec_start_index is earlier than brake_start_index
% Acc is from start_acc_index to const_speed_start
% const15 is from const_speed_start to dec_start_index
% Dec is from brake_start_index to end_brake_index
speed_threshold = 7/3.6; % unit m/s !!!!!will be modified later
brake_t_over_threshold = 1.5; % init s, !!!!!!will be mdified later
% acc_x = LinearAcceleration_x_filtered
% but acc_x and speed come from different timestamp and has different
% lenghth, so we need to make acc_x the same length as speed, considering
% their sampling frequency is very close, we just simply cut the longer
% part

    acc_x=test_slot_cell{i}.LinearAcceleration_x_filtered;

[dec_start_index, brake_start_index] = getBrakeIndex(end_brake_index, speed_threshold, test_slot_cell{i}.TimeStamp_imu, acc_x, Speed_from_lidar_synchronized, brake_t_over_threshold);
% now we have finished the same steps that we have done for task_id=1/2,
% the following will be something new, i.e detect slalom behavior.
% use function getSlalomIndices
% function [slalom_start_index, slalom_end_index] = getSlalomIndices(brake_start_index, time, st_rate, gyro_x, t_over_thsd, st_rate_thsd, gyro_x_thsd, max_duration)
t_over_thsd=3;%!!!!!!! will be modified later
st_rate_thsd=5; %!!!!!will be modified later
gyro_x_thsd=3; %!!!!!will be modified later
max_duration=5;%!!!!will be modified later

    time=test_slot_cell{i}.TimeStamp_imu;
    gyro_x=test_slot_cell{i}.AngularVelocity_x_filtered;
    st_rate=test_slot_cell{i}.steering_rate;

[slalom_start_index, slalom_end_index] = getSlalomIndices(brake_start_index, time, st_rate, gyro_x, t_over_thsd, st_rate_thsd, gyro_x_thsd, max_duration);
% recap again
% Acc is from start_acc_index to const_speed_start
% const7 is from const_speed_start to dec_start_index
% Dec is from brake_start_index to end_brake_index
% Slalom is from slalom_start_index to slalom_end_index

% segmentation check, if the segmentation is unreasonable, it doesn't make
% sense to continue further analysis
if (end_brake_index>brake_start_index && brake_start_index>=dec_start_index &&dec_start_index>const_speed_start &&const_speed_start>start_acc_index &&slalom_start_index>const_speed_start && slalom_end_index<end_brake_index )~=1
    error('segmentation is wrong, please check the code again');
end

% until now, we have Acc, Dec, Const7, slalom segmentation, according to paper Eval of a new narrow and tiltable electric cycle (e-trike) concept - final report
% we need to firstly extract :
% segmentation time [s] (for Acc, Dec,slalom)
% speed [m/s] (for Const7,slalom)
% steering rate [deg/s] (for Const7,slalom)
% roll rate [deg/s] (for Const7,slalom)
% steering angle [deg] (for Const7,slalom)
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.task_id = test_slot_cell{i}.task_id; %!!!!!!!will be added later
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.Acc_timerange=test_slot_cell{i}.TimeStamp_imu(start_acc_index:const_speed_start);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.Acc_time_start=test_slot_cell{i}.TimeStamp_imu(start_acc_index);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.Acc_time_end=test_slot_cell{i}.TimeStamp_imu(const_speed_start);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.Dec_timerange=test_slot_cell{i}.TimeStamp_imu(brake_start_index:end_brake_index);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.Dec_time_start=test_slot_cell{i}.TimeStamp_imu(brake_start_index);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.Dec_time_end=test_slot_cell{i}.TimeStamp_imu(end_brake_index);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.Slalom_timerange=test_slot_cell{i}.TimeStamp_imu(slalom_start_index:slalom_end_index);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.Slalom_time_start=test_slot_cell{i}.TimeStamp_imu(slalom_start_index);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.Slalom_time_end=test_slot_cell{i}.TimeStamp_imu(slalom_end_index);

segmentation_and_PI_cell{i}.Segmentation_signal_extraction.speed_range_const7=Speed_from_lidar_synchronized(const_speed_start:dec_start_index);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.speed_range_slalom=Speed_from_lidar_synchronized(slalom_start_index:slalom_end_index);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.steering_rate_range_const7=test_slot_cell{i}.steering_rate(const_speed_start:dec_start_index);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.roll_rate_range_const7=test_slot_cell{i}.AngularVelocity_x_filtered(const_speed_start:dec_start_index);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.steering_angle_range_const7=test_slot_cell{i}.SteeringAngle_filtered(const_speed_start:dec_start_index);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.steering_rate_range_slalom=test_slot_cell{i}.steering_rate(slalom_start_index:slalom_end_index);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.roll_rate_range_slalom=test_slot_cell{i}.AngularVelocity_x_filtered(slalom_start_index:slalom_end_index);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.steering_angle_range_slalom=test_slot_cell{i}.SteeringAngle_filtered(slalom_start_index:slalom_end_index);

% after extracting the signal, we need to computer performance indicator
% based on the signal we have extracted.
% time [s] (for Acc, Dec, for slalom)
% mean speed [m/s] (for Const7,for slalom)
% time delay between roll rate and steering rate [s] (for Const7.,for slalom)
% R^2 of linear fit of steering rate and roll rate [-] (for Const7,for slalom)
% mean absolute roll rate [deg/s] (for Const7,for slalom)
% mean absolute steering angle [deg] (for Const7,for slalom)
segmentation_and_PI_cell{i}.Performance_indicator.task_id=test_slot_cell{i}.task_id; %!!!!!!will be added later
segmentation_and_PI_cell{i}.Performance_indicator.Acc_time=test_slot_cell{i}.TimeStamp_imu(const_speed_start)-test_slot_cell{i}.TimeStamp_imu(start_acc_index);
segmentation_and_PI_cell{i}.Performance_indicator.Dec_time=test_slot_cell{i}.TimeStamp_imu(end_brake_index)-test_slot_cell{i}.TimeStamp_imu(brake_start_index);
segmentation_and_PI_cell{i}.Performance_indicator.Slalom_time=test_slot_cell{i}.TimeStamp_imu(slalom_end_index)-test_slot_cell{i}.TimeStamp_imu(slalom_start_index);
segmentation_and_PI_cell{i}.Performance_indicator.mean_speed_const7=mean(Speed_from_lidar_synchronized(const_speed_start:dec_start_index));
segmentation_and_PI_cell{i}.Performance_indicator.mean_speed_slalom=mean(Speed_from_lidar_synchronized(slalom_start_index:slalom_end_index));
% use function delaySteerGyro to compute time delay
% function value = delaySteerGyro(time, st_rate, gyro_x, segment_start, segment_end)
    time=test_slot_cell{i}.TimeStamp_imu;
    gyro_x=test_slot_cell{i}.AngularVelocity_x_filtered;
    st_rate=test_slot_cell{i}.steering_rate;
segmentation_and_PI_cell{i}.Performance_indicator.delay_between_rollrate_and_steeringrate_const7 = delaySteerGyro(time, st_rate, gyro_x, const_speed_start, dec_start_index);
segmentation_and_PI_cell{i}.Performance_indicator.delay_between_rollrate_and_steeringrate_slalom = delaySteerGyro(time, st_rate, gyro_x, slalom_start_index,slalom_end_index);

% use function r2SteerGyro to compute R^2 of linear fit of steering rate and roll rate
% function value = r2SteerGyro(st_rate, gyro_x, p_rate, segment_start, segment_end)
% p_rate = polyfit(st_rate,gyro_x,1); p_rate has 2 values inside, k(slope)
% and b(interception)
p_rate = polyfit(st_rate(const_speed_start:dec_start_index),gyro_x(const_speed_start:dec_start_index),1);
segmentation_and_PI_cell{i}.Performance_indicator.R2_between_rollrate_and_steeringrate_const7 = r2SteerGyro(st_rate, gyro_x, p_rate, const_speed_start, dec_start_index);
p_rate = polyfit(st_rate(slalom_start_index:slalom_end_index),gyro_x(slalom_start_index:slalom_end_index),1);
segmentation_and_PI_cell{i}.Performance_indicator.R2_between_rollrate_and_steeringrate_slalom = r2SteerGyro(st_rate, gyro_x, p_rate, slalom_start_index,slalom_end_index);

segmentation_and_PI_cell{i}.Performance_indicator.absolute_rollrate_const7=mean(abs(test_slot_cell{i}.AngularVelocity_x_filtered(const_speed_start:dec_start_index)));
segmentation_and_PI_cell{i}.Performance_indicator.absolute_steering_angle_const7=mean(abs(test_slot_cell{i}.SteeringAngle_filtered(const_speed_start:dec_start_index)));
segmentation_and_PI_cell{i}.Performance_indicator.absolute_rollrate_slalom=mean(abs(test_slot_cell{i}.AngularVelocity_x_filtered(slalom_start_index:slalom_end_index)));
segmentation_and_PI_cell{i}.Performance_indicator.absolute_steering_angle_slalom=mean(abs(test_slot_cell{i}.SteeringAngle_filtered(slalom_start_index:slalom_end_index)));

    end
end

save('Segmentation_and_PI.mat','segmentation_and_PI_cell');
disp('Good! Segmentation_and_PI result are saved');
