% this code should be run after Save_summarized_datatable.m has been run

% this code aims to split all data that is read from one bag file and
% contains several tasks into individual task and save those splitted
% indivisual tasks into different .mat files with different names

% the pre-request of this code is that: 
% number of lidar file == number of bike sensors file
% 1 participant 1 e-vehicle 3 tasks == 1 lidar bag file + 1 imu bag file
% in total, according to our experiment configuration, there should be
% % 30 participant 4 e-vehicle 3 tasks == 120 lidar bag file + 120 imu bag file

%%
%========#####################==========imu and adc part part=========######################===========
%Until now all the pre-processing of all signal has been
%finished,pre-processing includes but not limited to
%calibration,smoothing,frequency domain filtering.

% now we are going to split the whole bag file into different tasks
% according to flag_equals_1_time and how_any_tasks these two variables that
% we have got at the beginning
% Attention: this needs protocol to be strictly carried out during test that before each task starts(including the first task), the flag button is pressed, and after all tasks ends, stop button needs to be pressed in time
% otherwise if the raw data itself is too deviated from ideal, i.e what it
% should be, you can't split the tasks by code, you need to open them manually and cut by yourself.

% get the splitting indices for imu and adc signal
task_split_indices=nan(how_many_tasks,1);
for i=1:how_many_tasks
    temp=find(TimeStamp_imu>=flag_equals_1_time(i));
    task_split_indices(i)=temp(1);
end

% split the data and save them seperately in a cell, the variable name
% keeps the same, only content changed
tasks_split_cell=cell(how_many_tasks,1);
for i=1:how_many_tasks

    if i ~= how_many_tasks
        %because all speed_filtered, steeringangle_filtered and
        %steering_rate are computed based on the data after
        %synchronization, so TimeStamp_adc is already abondoned
        %tasks_split_cell{i}.TimeStamp_adc = TimeStamp_adc(task_split_indices(i):task_split_indices(i+1)-1);
        tasks_split_cell{i}.Speed_filtered = Speed_filtered(task_split_indices(i):task_split_indices(i+1)-1);
        tasks_split_cell{i}.SteeringAngle_filtered = SteeringAngle_filtered(task_split_indices(i):task_split_indices(i+1)-1);
        tasks_split_cell{i}.steering_rate = steering_rate(task_split_indices(i):task_split_indices(i+1)-1);
        
        tasks_split_cell{i}.TimeStamp_imu = TimeStamp_imu(task_split_indices(i):task_split_indices(i+1)-1);
        tasks_split_cell{i}.LinearAcceleration_x_filtered = LinearAcceleration_x_filtered(task_split_indices(i):task_split_indices(i+1)-1);
        tasks_split_cell{i}.AngularVelocity_x_filtered = AngularVelocity_x_filtered(task_split_indices(i):task_split_indices(i+1)-1);
        
        
    else
        %tasks_split_cell{i}.TimeStamp_adc = TimeStamp_adc(task_split_indices(i):task_split_indices(end));
        tasks_split_cell{i}.Speed_filtered = Speed_filtered(task_split_indices(i):end);
        tasks_split_cell{i}.SteeringAngle_filtered = SteeringAngle_filtered(task_split_indices(i):end);
        tasks_split_cell{i}.steering_rate = steering_rate(task_split_indices(i):end);
        
        tasks_split_cell{i}.TimeStamp_imu = TimeStamp_imu(task_split_indices(i):end);
        tasks_split_cell{i}.LinearAcceleration_x_filtered = LinearAcceleration_x_filtered(task_split_indices(i):end);
        tasks_split_cell{i}.AngularVelocity_x_filtered = AngularVelocity_x_filtered(task_split_indices(i):end);
        
    end
end

%========#####################==========save individual task part=========######################===========
save('individual_tasks_split.mat','tasks_split_cell');
disp('Good! Individual tasks have been seperated and saved in task')

clear