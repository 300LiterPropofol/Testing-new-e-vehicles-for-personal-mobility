% this code should run after Split_and_save_tasks.m has been run
% this code aims to cut out the real test slot out of the individual task
% that is already cut once, i.e this code is a second cut

% to be more specific, we need to cut the head and tail of each individual
% task and keep the test slot in the middle.

% in this experiment, we decided to press the lidar and bike sensors
% flagbutton like below
% lidar   .      .         .       .       .      .       .       .      .
% imu     .                        .                      .

% so we press lidar flag down when the participant fully stop at the
% starting line once, and press flag button down another time when the
% participant fully stop at the terminal line. These two flag signal is the
% key that we cut out test slot.

% there are so many variables in the workspace, first we clear them

close all

%%
% read in the firstly cleaned data that we already saved in
% individual_tasks_split
load individual_tasks_split.mat
how_many_tasks = length(tasks_split_cell);
test_slot_cell = cell(how_many_tasks,1); % initialization
for i = 1 : how_many_tasks
    task_temp = tasks_split_cell{i};
    middle_position_index = round(length(task_temp.Speed_filtered)/2); % the middle position of Speed data
    start_temp = find(task_temp.Speed_filtered(1:middle_position_index) <= 0.5);
    test_start_index_imu = start_temp(end);
    end_temp = find(task_temp.Speed_filtered(middle_position_index+1:end) <= 0.5);
    test_end_index_imu = end_temp(1)+middle_position_index;
    
    test_slot_cell{i}.task_id = i; % initialization
    test_slot_cell{i}.Speed_filtered = task_temp.Speed_filtered(test_start_index_imu:test_end_index_imu);
    test_slot_cell{i}.SteeringAngle_filtered = task_temp.SteeringAngle_filtered(test_start_index_imu:test_end_index_imu);
    test_slot_cell{i}.steering_rate = task_temp.steering_rate(test_start_index_imu:test_end_index_imu);
    test_slot_cell{i}.TimeStamp_imu = task_temp.TimeStamp_imu(test_start_index_imu:test_end_index_imu);
    test_slot_cell{i}.LinearAcceleration_x_filtered = task_temp.LinearAcceleration_x_filtered(test_start_index_imu:test_end_index_imu);
    test_slot_cell{i}.AngularVelocity_x_filtered = task_temp.AngularVelocity_x_filtered(test_start_index_imu:test_end_index_imu);
    

    

end
%========#####################==========savetest slot part=========######################===========
save('testslot_second_split.mat','test_slot_cell');
disp('Good! test slots have been second cut and saved')