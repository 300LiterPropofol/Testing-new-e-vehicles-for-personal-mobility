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
    
    
    % plot speed data from speed sensor
    figure;
    plot(test_slot_cell{i}.TimeStamp_imu,test_slot_cell{i}.Speed_filtered);
    xlabel('TimeStamp_imu (s)');
    ylabel('Speed (m/s)');
    title('Speed directly from speed sensor');
    
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
    if test_slot_cell{i}.task_id == 1 || test_slot_cell{i}.task_id == 2
% when task_id=1/2, use function getAccIndices
% function [start_acc_index, const_speed_start, end_brake_index] = getAccIndices(time, speed, min_speed_thsd, req_speed, min_t_over_thsd)
% time = TimeStamp_imu
% speed = Speed_filtered
min_speed_threshold = 0.5; % unit m/s !!!!!!!will be modified later
request_speed = 15; % unit m/s !!!!!!!will be modified later
min_t_over_threshold = 8; % unit s !!!!!!will be modified later
[start_acc_index, const_speed_start, end_brake_index] = getAccIndices(test_slot_cell{i}.TimeStamp_imu, test_slot_cell{i}.Speed_filtered, min_speed_threshold, request_speed, min_t_over_threshold);
% continue working on task_id=1/2, use function getBrakeIndex
% [dec_start_index, brake_start_index] = getBrakeIndex(end_brake_index, speed_thsd, time, acc_x, speed, brake_t_over_thsd)
% dec_start_index is earlier than brake_start_index?
% Acc is from start_acc_index to const_speed_start
% const15 is from const_speed_start to dec_start_index
% Dec is from brake_start_index to end_brake_index
speed_threshold = 14; % unit m/s !!!!!will be modified later
brake_t_over_threshold = 8; % init s, !!!!!!will be mdified later
acc_x = test_slot_cell{i}.LinearAcceleration_x_filtered;
[dec_start_index, brake_start_index] = getBrakeIndex(end_brake_index, speed_threshold, test_slot_cell{i}.TimeStamp_imu, acc_x, test_slot_cell{i}.Speed_filtered, brake_t_over_threshold);

% recap again
% Acc is from start_acc_index to const_speed_start
% const15 is from const_speed_start to dec_start_index
% Dec is from brake_start_index to end_brake_index

% segmentation check, if the segmentation is unreasonable, it doesn't make
% sense to continue further analysis
if (end_brake_index>brake_start_index && brake_start_index>=dec_start_index &&dec_start_index>const_speed_start &&const_speed_start>start_acc_index)~=1
    error('segmentation is wrong, please check the code again');
end

    % plot speed data from speed sensor
    figure;
    plot(test_slot_cell{i}.TimeStamp_imu,test_slot_cell{i}.Speed_filtered);
    hold on
    y_range = [1.3*min(test_slot_cell{i}.Speed_filtered) 1.3*max(test_slot_cell{i}.Speed_filtered)];
    plot([test_slot_cell{i}.TimeStamp_imu(start_acc_index) test_slot_cell{i}.TimeStamp_imu(start_acc_index)],y_range,'b');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(const_speed_start) test_slot_cell{i}.TimeStamp_imu(const_speed_start)],y_range,'k');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(dec_start_index) test_slot_cell{i}.TimeStamp_imu(dec_start_index)],y_range,'g');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(brake_start_index) test_slot_cell{i}.TimeStamp_imu(brake_start_index)],y_range,'y');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(end_brake_index) test_slot_cell{i}.TimeStamp_imu(end_brake_index)],y_range,'r');
    xlabel('TimeStamp_imu (s)');
    ylabel('Speed (m/s)');
    title('Speed directly from speed sensor');
    legend('signal','start acc index','const speed start','dec start index','brake start index','end brake index');
    
    % plot acc-x data
    figure;
    y_range = [1.3*min(test_slot_cell{i}.LinearAcceleration_x_filtered),1.3*max(test_slot_cell{i}.LinearAcceleration_x_filtered)];
    plot(test_slot_cell{i}.TimeStamp_imu,test_slot_cell{i}.LinearAcceleration_x_filtered);
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(start_acc_index) test_slot_cell{i}.TimeStamp_imu(start_acc_index)],y_range,'b');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(const_speed_start) test_slot_cell{i}.TimeStamp_imu(const_speed_start)],y_range,'k');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(dec_start_index) test_slot_cell{i}.TimeStamp_imu(dec_start_index)],y_range,'g');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(brake_start_index) test_slot_cell{i}.TimeStamp_imu(brake_start_index)],y_range,'y');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(end_brake_index) test_slot_cell{i}.TimeStamp_imu(end_brake_index)],y_range,'r');
    legend('signal','start acc index','const speed start','dec start index','brake start index','end brake index');
    xlabel('TimeStamp_imu (s)');
    ylabel('unit (m/s^2)');
    title('LinearAcceleration x check');
    
    % plot gyro-x data
    figure;
    y_range = [1.3*min(test_slot_cell{i}.AngularVelocity_x_filtered),1.3*max(test_slot_cell{i}.AngularVelocity_x_filtered)];
    plot(test_slot_cell{i}.TimeStamp_imu,test_slot_cell{i}.AngularVelocity_x_filtered);
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(start_acc_index) test_slot_cell{i}.TimeStamp_imu(start_acc_index)],y_range,'b');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(const_speed_start) test_slot_cell{i}.TimeStamp_imu(const_speed_start)],y_range,'k');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(dec_start_index) test_slot_cell{i}.TimeStamp_imu(dec_start_index)],y_range,'g');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(brake_start_index) test_slot_cell{i}.TimeStamp_imu(brake_start_index)],y_range,'y');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(end_brake_index) test_slot_cell{i}.TimeStamp_imu(end_brake_index)],y_range,'r');
    legend('signal','start acc index','const speed start','dec start index','brake start index','end brake index');
    xlabel('TimeStamp_imu (s)');
    ylabel('unit (rad/s)');
    title('AngularVelocity x check');
    
    % plot steering angle
    figure;
    y_range =[1.3*min(test_slot_cell{i}.SteeringAngle_filtered),1.3*max(test_slot_cell{i}.SteeringAngle_filtered)];
    plot(test_slot_cell{i}.TimeStamp_imu,test_slot_cell{i}.SteeringAngle_filtered);
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(start_acc_index) test_slot_cell{i}.TimeStamp_imu(start_acc_index)],y_range,'b');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(const_speed_start) test_slot_cell{i}.TimeStamp_imu(const_speed_start)],y_range,'k');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(dec_start_index) test_slot_cell{i}.TimeStamp_imu(dec_start_index)],y_range,'g');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(brake_start_index) test_slot_cell{i}.TimeStamp_imu(brake_start_index)],y_range,'y');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(end_brake_index) test_slot_cell{i}.TimeStamp_imu(end_brake_index)],y_range,'r');
    legend('signal','start acc index','const speed start','dec start index','brake start index','end brake index');
    xlabel('TimeStamp_imu (s)');
    ylabel('unit (deg)');
    title('Steering Angle check');
    
    % plot steering rate
    figure;
    plot(test_slot_cell{i}.TimeStamp_imu,test_slot_cell{i}.steering_rate);
    hold on
    y_range = [1.3*min(test_slot_cell{i}.steering_rate),1.3*max(test_slot_cell{i}.steering_rate)];
    plot([test_slot_cell{i}.TimeStamp_imu(start_acc_index) test_slot_cell{i}.TimeStamp_imu(start_acc_index)],y_range,'b');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(const_speed_start) test_slot_cell{i}.TimeStamp_imu(const_speed_start)],y_range,'k');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(dec_start_index) test_slot_cell{i}.TimeStamp_imu(dec_start_index)],y_range,'g');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(brake_start_index) test_slot_cell{i}.TimeStamp_imu(brake_start_index)],y_range,'y');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(end_brake_index) test_slot_cell{i}.TimeStamp_imu(end_brake_index)],y_range,'r');
    legend('signal','start acc index','const speed start','dec start index','brake start index','end brake index');
    xlabel('TimeStamp_imu (s)');
    ylabel('unit (deg/s)');
    title('Steering rate check');


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

segmentation_and_PI_cell{i}.Segmentation_signal_extraction.speed_range_const15=test_slot_cell{i}.Speed_filtered(const_speed_start:dec_start_index);

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
segmentation_and_PI_cell{i}.Performance_indicator.mean_speed_const15=mean(test_slot_cell{i}.Speed_filtered(const_speed_start:dec_start_index));
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
min_speed_threshold = 0.5; % unit m/s !!!!!!!will be modified later
request_speed = 6.8; % unit m/s !!!!!!!will be modified later
min_t_over_threshold = 8; % unit s !!!!!!will be modified later
[start_acc_index, const_speed_start, end_brake_index] = getAccIndices(test_slot_cell{i}.TimeStamp_imu, test_slot_cell{i}.Speed_filtered, min_speed_threshold, request_speed, min_t_over_threshold);
% continue working on task_id=3, use function getBrakeIndex
% [dec_start_index, brake_start_index] = getBrakeIndex(end_brake_index, speed_thsd, time, acc_x, speed, brake_t_over_thsd)
% dec_start_index is earlier than brake_start_index
% Acc is from start_acc_index to const_speed_start
% const15 is from const_speed_start to dec_start_index
% Dec is from brake_start_index to end_brake_index
speed_threshold = 6.8; % unit m/s !!!!!will be modified later
brake_t_over_threshold = 1.5; % init s, !!!!!!will be mdified later
% acc_x = LinearAcceleration_x_filtered
% but acc_x and speed come from different timestamp and has different
% lenghth, so we need to make acc_x the same length as speed, considering
% their sampling frequency is very close, we just simply cut the longer
% part

    acc_x=test_slot_cell{i}.LinearAcceleration_x_filtered;

[dec_start_index, brake_start_index] = getBrakeIndex(end_brake_index, speed_threshold, test_slot_cell{i}.TimeStamp_imu, acc_x, test_slot_cell{i}.Speed_filtered, brake_t_over_threshold);
% now we have finished the same steps that we have done for task_id=1/2,
% the following will be something new, i.e detect slalom behavior.
% use function getSlalomIndices
% function [slalom_start_index, slalom_end_index] = getSlalomIndices(brake_start_index, time, st_rate, gyro_x, t_over_thsd, st_rate_thsd, gyro_x_thsd, max_duration)
t_over_thsd=3;%!!!!!!! will be modified later
st_rate_thsd=300; %!!!!!will be modified later
gyro_x_thsd=0.2; %!!!!!will be modified later
max_duration=10;%!!!!will be modified later

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

    % plot speed data from speed sensor
    figure;
    plot(test_slot_cell{i}.TimeStamp_imu,test_slot_cell{i}.Speed_filtered);
    hold on
    y_range = [1.3*min(test_slot_cell{i}.Speed_filtered) 1.3*max(test_slot_cell{i}.Speed_filtered)];
    plot([test_slot_cell{i}.TimeStamp_imu(start_acc_index) test_slot_cell{i}.TimeStamp_imu(start_acc_index)],y_range,'b');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(const_speed_start) test_slot_cell{i}.TimeStamp_imu(const_speed_start)],y_range,'k');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(dec_start_index) test_slot_cell{i}.TimeStamp_imu(dec_start_index)],y_range,'g');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(brake_start_index) test_slot_cell{i}.TimeStamp_imu(brake_start_index)],y_range,'y');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(end_brake_index) test_slot_cell{i}.TimeStamp_imu(end_brake_index)],y_range,'r');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(slalom_start_index) test_slot_cell{i}.TimeStamp_imu(slalom_start_index)],y_range,'m--');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(slalom_end_index) test_slot_cell{i}.TimeStamp_imu(slalom_end_index)],y_range,'c--');
    legend('signal','start acc index','const speed start','dec start index','brake start index','end brake index','slalom start index','slalom end index');
    xlabel('TimeStamp_imu (s)');
    ylabel('Speed (m/s)');
    title('Speed directly from speed sensor');
    
    % plot acc-x data
    figure;
    y_range = [1.3*min(test_slot_cell{i}.LinearAcceleration_x_filtered),1.3*max(test_slot_cell{i}.LinearAcceleration_x_filtered)];
    plot(test_slot_cell{i}.TimeStamp_imu,test_slot_cell{i}.LinearAcceleration_x_filtered);
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(start_acc_index) test_slot_cell{i}.TimeStamp_imu(start_acc_index)],y_range,'b');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(const_speed_start) test_slot_cell{i}.TimeStamp_imu(const_speed_start)],y_range,'k');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(dec_start_index) test_slot_cell{i}.TimeStamp_imu(dec_start_index)],y_range,'g');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(brake_start_index) test_slot_cell{i}.TimeStamp_imu(brake_start_index)],y_range,'y');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(end_brake_index) test_slot_cell{i}.TimeStamp_imu(end_brake_index)],y_range,'r');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(slalom_start_index) test_slot_cell{i}.TimeStamp_imu(slalom_start_index)],y_range,'m--');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(slalom_end_index) test_slot_cell{i}.TimeStamp_imu(slalom_end_index)],y_range,'c--');
    legend('signal','start acc index','const speed start','dec start index','brake start index','end brake index','slalom start index','slalom end index');
    xlabel('TimeStamp_imu (s)');
    ylabel('unit (m/s^2)');
    title('LinearAcceleration x check');
    
    % plot gyro-x data
    figure;
    y_range = [1.3*min(test_slot_cell{i}.AngularVelocity_x_filtered),1.3*max(test_slot_cell{i}.AngularVelocity_x_filtered)];
    plot(test_slot_cell{i}.TimeStamp_imu,test_slot_cell{i}.AngularVelocity_x_filtered);
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(start_acc_index) test_slot_cell{i}.TimeStamp_imu(start_acc_index)],y_range,'b');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(const_speed_start) test_slot_cell{i}.TimeStamp_imu(const_speed_start)],y_range,'k');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(dec_start_index) test_slot_cell{i}.TimeStamp_imu(dec_start_index)],y_range,'g');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(brake_start_index) test_slot_cell{i}.TimeStamp_imu(brake_start_index)],y_range,'y');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(end_brake_index) test_slot_cell{i}.TimeStamp_imu(end_brake_index)],y_range,'r');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(slalom_start_index) test_slot_cell{i}.TimeStamp_imu(slalom_start_index)],y_range,'m--');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(slalom_end_index) test_slot_cell{i}.TimeStamp_imu(slalom_end_index)],y_range,'c--');
    legend('signal','start acc index','const speed start','dec start index','brake start index','end brake index','slalom start index','slalom end index');
    xlabel('TimeStamp_imu (s)');
    ylabel('unit (rad/s)');
    title('AngularVelocity x check');
    
    % plot steering angle
    figure;
    y_range =[1.3*min(test_slot_cell{i}.SteeringAngle_filtered),1.3*max(test_slot_cell{i}.SteeringAngle_filtered)];
    plot(test_slot_cell{i}.TimeStamp_imu,test_slot_cell{i}.SteeringAngle_filtered);
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(start_acc_index) test_slot_cell{i}.TimeStamp_imu(start_acc_index)],y_range,'b');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(const_speed_start) test_slot_cell{i}.TimeStamp_imu(const_speed_start)],y_range,'k');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(dec_start_index) test_slot_cell{i}.TimeStamp_imu(dec_start_index)],y_range,'g');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(brake_start_index) test_slot_cell{i}.TimeStamp_imu(brake_start_index)],y_range,'y');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(end_brake_index) test_slot_cell{i}.TimeStamp_imu(end_brake_index)],y_range,'r');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(slalom_start_index) test_slot_cell{i}.TimeStamp_imu(slalom_start_index)],y_range,'m--');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(slalom_end_index) test_slot_cell{i}.TimeStamp_imu(slalom_end_index)],y_range,'c--');
    legend('signal','start acc index','const speed start','dec start index','brake start index','end brake index','slalom start index','slalom end index');
    xlabel('TimeStamp_imu (s)');
    ylabel('unit (deg)');
    title('Steering Angle check');
    
    % plot steering rate
    figure;
    plot(test_slot_cell{i}.TimeStamp_imu,test_slot_cell{i}.steering_rate);
    hold on
    y_range = [1.3*min(test_slot_cell{i}.steering_rate),1.3*max(test_slot_cell{i}.steering_rate)];
    plot([test_slot_cell{i}.TimeStamp_imu(start_acc_index) test_slot_cell{i}.TimeStamp_imu(start_acc_index)],y_range,'b');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(const_speed_start) test_slot_cell{i}.TimeStamp_imu(const_speed_start)],y_range,'k');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(dec_start_index) test_slot_cell{i}.TimeStamp_imu(dec_start_index)],y_range,'g');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(brake_start_index) test_slot_cell{i}.TimeStamp_imu(brake_start_index)],y_range,'y');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(end_brake_index) test_slot_cell{i}.TimeStamp_imu(end_brake_index)],y_range,'r');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(slalom_start_index) test_slot_cell{i}.TimeStamp_imu(slalom_start_index)],y_range,'m--');
    hold on
    plot([test_slot_cell{i}.TimeStamp_imu(slalom_end_index) test_slot_cell{i}.TimeStamp_imu(slalom_end_index)],y_range,'c--');
    legend('signal','start acc index','const speed start','dec start index','brake start index','end brake index','slalom start index','slalom end index');
    xlabel('TimeStamp_imu (s)');
    ylabel('unit (deg/s)');
    title('Steering rate check');



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

segmentation_and_PI_cell{i}.Segmentation_signal_extraction.speed_range_const7=test_slot_cell{i}.Speed_filtered(const_speed_start:dec_start_index);
segmentation_and_PI_cell{i}.Segmentation_signal_extraction.speed_range_slalom=test_slot_cell{i}.Speed_filtered(slalom_start_index:slalom_end_index);
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
segmentation_and_PI_cell{i}.Performance_indicator.mean_speed_const7=mean(test_slot_cell{i}.Speed_filtered(const_speed_start:dec_start_index));
segmentation_and_PI_cell{i}.Performance_indicator.mean_speed_slalom=mean(test_slot_cell{i}.Speed_filtered(slalom_start_index:slalom_end_index));
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
