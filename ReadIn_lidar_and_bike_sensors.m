% Author: Di Xue dixu@student.chalmers.se
% last modification date: 2019-12-17
%
% read me:
% You need always pay attention to the data type you get in raw bag file
% and the method you use to transfer them. 
% The raw data ma y be in format uint32 uint8 single and so on and you are aiming
% to finally transfer all data into double for further calculation
% It is highly possible to be problematic when doing double() or *10^-9 etc
% on these data, always check output on stages, output includes but not
% limited to timestamp, lidar range...
% Use cast() if the object data form
% has more bits than the current one, use typecast() if the object data form
% has less bits than the current one. 
% Always make sure you don't accidentally transfer your data into integer.
% Always make sure all your temporary variables in workspace are in nx1 format,
% instead of 1xn format, otherwise this can cause a lot of calculation trouble
% afterwards


%%
%========#####################==========lidar part=========######################===========
%============================chapter 1 import data, get input for further research=================================%
%extract bag to structure

%Input file name, read raw data which is in .bag format into matlab workspace 
bag = rosbag(lidar_filename); %data type: BagSelection

%select lidar data,and generate an array of cells where each cell represent
%one timestamp's information. 

%Information includes MessageType, Header,Ranges,etc.

% Information content depends on what we select, now we select '/scan'.

bagSelection_lidar = select(bag,'Topic','/scan'); %data type: BagSelection
msgStructs_lidar = readMessages(bagSelection_lidar,'DataFormat','struct'); %data type: nX1 cell, n means how many frames does this lidar dataset have

%use Header.Stamp.Sec, Header.Stamp.Nsec to get TimeStamp
frame_number_lidar = bagSelection_lidar.NumMessages;
timestart_lidar = cast(msgStructs_lidar{1}.Header.Stamp.Sec,'double') + cast(msgStructs_lidar{1}.Header.Stamp.Nsec,'double')*10^-9;
TimeStamp_lidar=nan(frame_number_lidar,1); % initialization
for i = 1:frame_number_lidar
    time_current= cast(msgStructs_lidar{i}.Header.Stamp.Sec,'double') + cast(msgStructs_lidar{i}.Header.Stamp.Nsec,'double')*10^-9;
    % msgStructs_lidar{i}.Header.Stamp.Sec has unit in seconds,
    % msgStructs_lidar{i}.Header.Stamp.Nsec has unit in nanoseconds,so our
    % TimeStamp should be in unit seconds
    TimeStamp_lidar(i)=time_current-timestart_lidar;
end
%%
%select flagbutton_pressed data, because we plan to take several tasks in
%protocol together and save them in one bag file, so we need to get the
%flag change and extract the timestamp when the flag change, in order to
%split different tasks afetrwards.
bagSelection_buttonflag_lidar = select(bag,'Topic','/flagbutton_pressed');
msgStructs_buttonflag_lidar = readMessages(bagSelection_buttonflag_lidar,'DataFormat','struct');
%use Header.Stamp.Sec, Header.Stamp.Nsec to get TimeStamp
frame_number_buttonflag_lidar = bagSelection_buttonflag_lidar.NumMessages;
timestart_buttonflag = cast(msgStructs_buttonflag_lidar{1}.Header.Stamp.Sec,'double') + cast(msgStructs_buttonflag_lidar{1}.Header.Stamp.Nsec,'double')*10^-9;
TimeStamp_lidarflag=nan(frame_number_buttonflag_lidar,1); % initialization
flag = nan(frame_number_buttonflag_lidar,1);
for i = 1:frame_number_buttonflag_lidar
    time_current= cast(msgStructs_buttonflag_lidar{i}.Header.Stamp.Sec,'double') + cast(msgStructs_buttonflag_lidar{i}.Header.Stamp.Nsec,'double')*10^-9;
    % msgStructs_buttonflag{i}.Header.Stamp.Sec has unit in seconds,
    % msgStructs_buttonflag{i}.Header.Stamp.Nsec has unit in nanoseconds,so our
    % TimeStamp should be in unit seconds
    TimeStamp_lidarflag(i)=time_current-timestart_buttonflag;
    flag(i)=msgStructs_buttonflag_lidar{i}.Data;
end
% what to notice is, the configuration in ROS is that when one presses the
% button, at that timestamp, it turns into flag=1, and the next timestamp
% returns to flag=0 immediately, so if you plot the flag value corresponding
% to TimeStamp_flagbutton, it should be like
%1           .      .      .        .
%0___________ ______ ______ ________ __________
% so, a bag file should be cut exactly at the flag=1 point and divide into
% deperate tasks for further analysis

% find flag=1 points and save the time of them
temp_i=[];
flag_equals_1_time_lidar=TimeStamp_lidarflag(flag==1);
for i=2:length(flag_equals_1_time_lidar)
    if flag_equals_1_time_lidar(i)-flag_equals_1_time_lidar(i-1)<1
        temp_i=[temp_i,i];
    end
end
flag_equals_1_time_lidar(temp_i)=[];
how_many_tasks_lidar = length(flag_equals_1_time_lidar)/3;
%%
% extract lidar data, lidar data is recorded in polar coordinates, which
% means at each timestamp, at each resolution angle, there is a value that
% represent the lidarrange. The lidar angle range is fixed (190
% degree),resolution is fixed (1521 angle values)
% So the result of this extraction step should be

%               angle_resolution(1) ---------------angle_resolution(1521)
% timestamp(1) 
% timestamp(2)  
% |
% |
% timestamp(frame_number_lidar)

lidar_angles_num = 1521;
lidar_viewrange = 190 ; % in degrees
lidar_angles = linspace((90 - lidar_viewrange/2)*pi/180,(90 + lidar_viewrange/2)*pi/180,lidar_angles_num); % in rad 
% Attention: pay attention that linspace() result in 1xn array, 
% you need to transfer it into nx1, 
% otherwise you will end up in big memory leakage black screen problem.
lidar_angles = lidar_angles';
% transfer the polar coordinates into cartersian coordinates,pol2cart()
% function needs theta in radius unit, which we have already satisfied. 
% x and y will be saved into different arrays due to the configuration of
% pol2cart() function.
cartersian_coordinate = cell(frame_number_lidar,1); %initialization
for i = 1:frame_number_lidar
    range_array_current_frame=cast(msgStructs_lidar{i}.Ranges,'double');
    [x_array_currentframe,y_array_currentframe]=pol2cart(lidar_angles,range_array_current_frame);
    cartersian_coordinate{i}= [x_array_currentframe,y_array_currentframe];
end
% until now, all coordinates transformation has been finished and saved in
% a nx1 cell format. n=number of lidar data frame

% the next step is to cluster the dot cloud and implement object
% detection, object detection is a image analysis concept, it is only
% responsible for individual frame/image,it locates the object and send out
% the location information of all cloud dots which belong to the object cluster.

%%
% select the testing field out before applying dbscan to all lidar point
% cloud, this will significantly reduce unnecessary computation and 
% make it easier to detect where is our e-bike is in the lidar point cloud
% because our test field area is a limited rectangular square, so we can
% apply 4 thresholds that represent left,right,up, down borderline
% resprctively and wipe all the points that lie outside the borderline.
cartersian_coordinate_interested = cartersian_coordinate; % initialization
% set borderline threshold, all thresholds are x or y in cartersian
% coordinates and unit is meter
up_threshold = 6.5; % borderline for y!!!!!!!!!!! modeified later
down_threshold = 0; % borderline for y!!!!!!!!!!! modeified later
left_threshold = -26; % borderline for x!!!!!!!!!!! modeified later
right_threshold = 35; % borderline for x!!!!!!!!!!! modeified later
for i=1:frame_number_lidar
    x_y_coordinatetemp=cartersian_coordinate_interested{i}; % data is 1521x2 format, the first column is x, the second column is y
    delete_coordinates_indices = find((x_y_coordinatetemp(:,2)>up_threshold | x_y_coordinatetemp(:,2)<down_threshold | x_y_coordinatetemp(:,1)<left_threshold | x_y_coordinatetemp(:,1)>right_threshold)==1);
    x_y_coordinatetemp(delete_coordinates_indices,:)=[];
    cartersian_coordinate_interested{i} = x_y_coordinatetemp;
end
%%
%========#####################==========bike sensors part=========######################===========
%Input file name, read raw data which is in .bag format into matlab workspace 
bag = rosbag(bike_sensors_filename); %data type: BagSelection

%select downbutton_pressed data, because we plan to take several tasks in
%protocol together and save them in one bag file, so we need to get the
%flag change and extract the timestamp when the flag change, in order to
%split different tasks afetrwards.
bagSelection_buttonflag = select(bag,'Topic','/downbutton_pressed');
msgStructs_buttonflag = readMessages(bagSelection_buttonflag,'DataFormat','struct');
%use Header.Stamp.Sec, Header.Stamp.Nsec to get TimeStamp
frame_number_buttonflag = bagSelection_buttonflag.NumMessages;
timestart_buttonflag = cast(msgStructs_buttonflag{1}.Header.Stamp.Sec,'double') + cast(msgStructs_buttonflag{1}.Header.Stamp.Nsec,'double')*10^-9;
TimeStamp_buttonflag=nan(frame_number_buttonflag,1); % initialization
flag = nan(frame_number_buttonflag,1);
for i = 1:frame_number_buttonflag
    time_current= cast(msgStructs_buttonflag{i}.Header.Stamp.Sec,'double') + cast(msgStructs_buttonflag{i}.Header.Stamp.Nsec,'double')*10^-9;
    % msgStructs_buttonflag{i}.Header.Stamp.Sec has unit in seconds,
    % msgStructs_buttonflag{i}.Header.Stamp.Nsec has unit in nanoseconds,so our
    % TimeStamp should be in unit seconds
    TimeStamp_buttonflag(i)=time_current-timestart_buttonflag;
    flag(i)=msgStructs_buttonflag{i}.Flag;
end
% what to notice is, the configuration in ROS is that when one presses the
% button, at that timestamp, it turns into flag=1, and the next timestamp
% returns to flag=0 immediately, so if you plot the flag value corresponding
% to TimeStamp_flagbutton, it should be like
%1           .      .      .        .
%0___________ ______ ______ ________ __________
% so, a bag file should be cut exactly at the flag=1 point and divide into
% deperate tasks for further analysis

% find flag=1 points and save the time of them
temp_i=[];
flag_equals_1_time=TimeStamp_buttonflag(flag==1);
for i=2:length(flag_equals_1_time)
    if flag_equals_1_time(i)-flag_equals_1_time(i-1)<1
        temp_i=[temp_i,i];
    end
end
flag_equals_1_time(temp_i)=[];
how_many_tasks = length(flag_equals_1_time);
%%
%select imu data,and generate an array of cells where each cell represent
%one timestamp's information. 

%Information includes MessageType, Header,
%Orientation, OrientationCovariance, AngularVelocity,
%AngularVelocityCovariance, LinearAcceleration,
%LinearAccelerationCovariance

% Information content depends on what we select, now we select '/imu/data'.

bagSelection_imu = select(bag,'Topic','/imu/data'); %data type: BagSelection
msgStructs_imu = readMessages(bagSelection_imu,'DataFormat','struct'); %data type: nX1 cell

%%
%extract data we want which are: Header.Stamp.Sec, Header.Stamp.Nsec
%Orientation.X, Orientation.Y, Orientation.Z, Orientation.W,
%AngularVelocity.X, AngularVelocity.Y, AngularVelocity.Z,
%LinearAcceleration.X, LinearAcceleration.Y, LinearAcceleration.Z

%use Header.Stamp.Sec, Header.Stamp.Nsec to get TimeStamp
frame_number_imu = bagSelection_imu.NumMessages;
timestart_imu = cast(msgStructs_imu{1}.Header.Stamp.Sec,'double') + cast(msgStructs_imu{1}.Header.Stamp.Nsec,'double')*10^-9;
TimeStamp_imu=nan(frame_number_imu,1); % initialization
for i = 1:frame_number_imu
    time_current= cast(msgStructs_imu{i}.Header.Stamp.Sec,'double') + cast(msgStructs_imu{i}.Header.Stamp.Nsec,'double')*10^-9;
    % msgStructs_imu{i}.Header.Stamp.Sec has unit in seconds,
    % msgStructs_imu{i}.Header.Stamp.Nsec has unit in nanoseconds,so our
    % TimeStamp should be in unit seconds
    TimeStamp_imu(i)=time_current-timestart_imu;
end
% TimeStamp will be var1 in our later signal table
%%
%extract Orientation.X, Orientation.Y, Orientation.Z, Orientation.W
Orientation =nan(frame_number_imu,4); % column order will be in Orientation.X, Orientation.Y, Orientation.Z, Orientation.W
yaw = nan(frame_number_imu,1); % yaw angle in radians, means rotation around z axis
pitch = nan(frame_number_imu,1); % pitch angle in randians, means rotation around y axis
roll = nan(frame_number_imu,1); % roll angle in radians, means rotation around x axis

for i=1:frame_number_imu
    Orientation(i,1)=msgStructs_imu{i}.Orientation.X;
    Orientation(i,2)=msgStructs_imu{i}.Orientation.Y;
    Orientation(i,3)=msgStructs_imu{i}.Orientation.Z;
    Orientation(i,4)=msgStructs_imu{i}.Orientation.W;
    % math background, q means quaternion q = w + xi + yj + zk
    % MATLAB code background, quaternion(A,B,C,D)=A + Bi + cj + Dk
    quaternion_construction = quaternion(Orientation(i,4),Orientation(i,1),Orientation(i,2),Orientation(i,3));
    euler_rotation = euler(quaternion_construction,'ZYX','frame'); % the output will be in order of yaw, pitch, roll, unit is radians
    yaw(i)=euler_rotation(1); % unit radians
    pitch(i)=euler_rotation(2); % unit radians
    roll(i)=euler_rotation(3);  % unit radians
end


%%
% extract AngularVelocity.X, AngularVelocity.Y, AngularVelocity.Z
AngularVelocity=nan(frame_number_imu,3); % column order will be in AngularVelocity.X, AngularVelocity.Y, AngularVelocity.Z
for i=1:frame_number_imu
    AngularVelocity(i,1)=-msgStructs_imu{i}.AngularVelocity.Y;% this is because sensor installation direction does not correspond to global coordiantes system
    AngularVelocity(i,2)=msgStructs_imu{i}.AngularVelocity.X;
    AngularVelocity(i,3)=-msgStructs_imu{i}.AngularVelocity.Z; 
end
%%
% extract LinearAcceleration.X, LinearAcceleration.Y, LinearAcceleration.Z
LinearAcceleration=nan(frame_number_imu,3); %column oreder will be in LinearAcceleration.X, LinearAcceleration.Y, LinearAcceleration.Z
for i=1:frame_number_imu
    LinearAcceleration(i,1)=-msgStructs_imu{i}.LinearAcceleration.Y;
    LinearAcceleration(i,2)=msgStructs_imu{i}.LinearAcceleration.X;
    LinearAcceleration(i,3)=-msgStructs_imu{i}.LinearAcceleration.Z;
end
%%
% make all signal from imu into a table
temp=table(TimeStamp_imu,Orientation,yaw,pitch,roll,AngularVelocity,LinearAcceleration);
SignalTable_imu = splitvars(temp);
SignalTable_imu.Properties.VariableNames{'Orientation_1'} = 'Orientation_X';
SignalTable_imu.Properties.VariableNames{'Orientation_2'} = 'Orientation_Y';
SignalTable_imu.Properties.VariableNames{'Orientation_3'} = 'Orientation_Z';
SignalTable_imu.Properties.VariableNames{'Orientation_4'} = 'Orientation_W';
SignalTable_imu.Properties.VariableNames{'AngularVelocity_1'} = 'AngularVelocity_X';
SignalTable_imu.Properties.VariableNames{'AngularVelocity_2'} = 'AngularVelocity_Y';
SignalTable_imu.Properties.VariableNames{'AngularVelocity_3'} = 'AngularVelocity_Z';
SignalTable_imu.Properties.VariableNames{'LinearAcceleration_1'} = 'LinearAcceleration_X';
SignalTable_imu.Properties.VariableNames{'LinearAcceleration_2'} = 'LinearAcceleration_Y';
SignalTable_imu.Properties.VariableNames{'LinearAcceleration_3'} = 'LinearAcceleration_Z';
%%
%select adc data,and generate an array of cells where each cell represent
%one timestamp's information. 

%Information includes Timestamp,SteeringAngle and speed

% Information content depends on what we select, now we select '/adc'.

bagSelection_adc = select(bag,'Topic','/adc'); %data type: BagSelection
msgStructs_adc = readMessages(bagSelection_adc,'DataFormat','struct'); %data type: nX1 cell
frame_number_adc = bagSelection_adc.NumMessages;
%%
TimeStamp_adc = nan(frame_number_adc,1); % initialization
%use Header.Stamp.Sec, Header.Stamp.Nsec to get TimeStamp
timestart_adc = cast(msgStructs_adc{1}.Header.Stamp.Sec,'double') + cast(msgStructs_adc{1}.Header.Stamp.Nsec,'double')*10^-9;
for i = 1:frame_number_adc
    time_current= cast(msgStructs_adc{i}.Header.Stamp.Sec,'double') + cast(msgStructs_adc{i}.Header.Stamp.Nsec,'double')*10^-9;           
    % msgStructs_adc{i}.Header.Stamp.Sec has unit in seconds,
    % msgStructs_adc{i}.Header.Stamp.Nsec has unit in nanoseconds,so our
    % TimeStamp should be in unit seconds
    TimeStamp_adc(i)=time_current-timestart_adc;
end
%%
% extract raw steering angle
SteeringAngle_raw =nan(frame_number_adc,1); 
for i=1:frame_number_adc
    SteeringAngle_raw(i)=cast(msgStructs_adc{i}.Data(1),'double');
end

SignalTable_adc=table(TimeStamp_adc,SteeringAngle_raw);

%until now, all signal data extraction has been completed, all raw data as input has
%been at hand, in the following we are going to synchronize imu and adc
%data, the plan is to keep imu data as unchanged, and map adc data to the
%same timestamp as imu, using interp1. After this, TimeStamp_adc is
%abandoned.
SteeringAngle_synchronized = interp1(TimeStamp_adc,SteeringAngle_raw,TimeStamp_imu,'linear','extrap');

temp = table(SteeringAngle_synchronized);
SignalTable_ADCIMU_synchronized = [SignalTable_imu temp];

%%
%until now, all signal data are synchronized , in the following we are going to process the data, this part
%is basically reusing or modifying Alexander Rasch's code.

%=======================chapter 2 filtering and processing the data===============================%
% first of all, the steering angle and speed needs calibration
% !!!!!!!! polyfit result k and b need to be added, k is straight line
% slope, b is straight line intercept.
% !!!!!!!! will modify calibrateSpeed.m and calibrateSteerAngle.m to get k
% and b in the future
% !!!!!!!! change SteeringAngle and Speed in work space into calibrated values
% !!!!!!!! update SignalTable_adc
k_st= 1;% slope in steering angle calibration
b_st= 0;% intercept in steering angle calibration

SteeringAngle_calibrated =k_st*SteeringAngle_synchronized+b_st;

%"All data is forward and reverse low pass filtered with a cut-off
%frequency of 7.5 Hz except for the speed data 
%which is filtered with a cut-off frequency of 2.5 Hz.
%The steering angle data is subtracted with its mean over time 
%to reduce the influence of any possible drift or belt slip effect." (!!!!I do not think in our turning big circle case signal minus mean is reseanable)
%the content inside quatations come from Rasch, A., Dozza, M., & Rasch, A. (2016). Evaluation of a new narrow and tiltable electric tricycle ( e-trike ) concept. 1–29.
% !!!!!!!!! fs means sampling frequecy,temporarily set at 250
% !!!!!!!!! cutoff frequency using Alex original value now
fs=250;
SteeringAngle_filtered_temp = lpFilterData(SteeringAngle_calibrated, 7.5, fs);
SteeringAngle_filtered = SteeringAngle_filtered_temp - mean(SteeringAngle_filtered_temp);
SteeringAngle_filtered = smoothdata(SteeringAngle_filtered,'gaussian',200);
LinearAcceleration_x_filtered = lpFilterData(LinearAcceleration(:, 1), 7.5, fs);
LinearAcceleration_x_filtered = smoothdata(LinearAcceleration_x_filtered,'gaussian',200);
LinearAcceleration_y_filtered = lpFilterData(LinearAcceleration(:, 2), 7.5, fs);
LinearAcceleration_y_filtered = smoothdata(LinearAcceleration_y_filtered,'gaussian',200);
LinearAcceleration_z_filtered = lpFilterData(LinearAcceleration(:, 3), 7.5, fs);
LinearAcceleration_z_filtered = smoothdata(LinearAcceleration_z_filtered,'gaussian',200);
AngularVelocity_x_filtered = lpFilterData(AngularVelocity(:, 1), 7.5, fs);
AngularVelocity_x_filtered = smoothdata(AngularVelocity_x_filtered,'gaussian',200);
AngularVelocity_y_filtered = lpFilterData(AngularVelocity(:, 2), 7.5, fs);
AngularVelocity_y_filtered = smoothdata(AngularVelocity_y_filtered,'gaussian',200);
AngularVelocity_z_filtered = lpFilterData(AngularVelocity(:, 3), 7.5, fs);
AngularVelocity_z_filtered = smoothdata(AngularVelocity_z_filtered,'gaussian',200);

steering_rate = getSteerRate(TimeStamp_imu, SteeringAngle_filtered);
SignalTable_after_preprocessing=table(TimeStamp_imu,SteeringAngle_filtered,steering_rate,LinearAcceleration_x_filtered,LinearAcceleration_y_filtered,LinearAcceleration_z_filtered,AngularVelocity_x_filtered,AngularVelocity_y_filtered,AngularVelocity_z_filtered);

%%
%========#####################==========how_many_tasks check part=========######################===========
if how_many_tasks == how_many_tasks_lidar
    disp('Good! flag button press number is the same in bike sensor and Lidar');
else
    error('Error: flag button press number is NOT the same in bike sensor and Lidar');
end