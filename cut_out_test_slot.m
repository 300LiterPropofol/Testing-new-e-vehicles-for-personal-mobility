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
    test_start_time_lidar = task_temp.flag_equals_1_lidar_testslot(1);
    test_end_time_lidar = task_temp.flag_equals_1_lidar_testslot(2);
    time_delay = tasks_split_cell{i}.timegap_lidar_minus_imu;
    test_start_time_imu = task_temp.TimeStamp_imu(1)+ test_start_time_lidar -task_temp.TimeStamp_lidar(1);
    test_end_time_imu = task_temp.TimeStamp_imu(1) + test_end_time_lidar - task_temp.TimeStamp_lidar(1);
    temp=find(task_temp.TimeStamp_lidar>=test_start_time_lidar);
    test_start_index_lidar = temp(1);
    temp = find(task_temp.TimeStamp_lidar>=test_end_time_lidar);
    test_end_index_lidar = temp(1);
    temp = find(task_temp.TimeStamp_imu>=test_start_time_imu);
    test_start_index_imu = temp(1);
    temp = find(task_temp.TimeStamp_imu>=test_end_time_imu);
    test_end_index_imu = temp(1);
    
    test_slot_cell{i}.task_id = i; % initialization

    test_slot_cell{i}.TimeStamp_imu = task_temp.TimeStamp_imu(test_start_index_imu:test_end_index_imu);
    test_slot_cell{i}.LinearAcceleration_x_filtered = task_temp.LinearAcceleration_x_filtered(test_start_index_imu:test_end_index_imu);
    test_slot_cell{i}.AngularVelocity_x_filtered = task_temp.AngularVelocity_x_filtered(test_start_index_imu:test_end_index_imu);
    test_slot_cell{i}.TimeStamp_lidar = task_temp.TimeStamp_lidar(test_start_index_lidar:test_end_index_lidar)-time_delay;
    test_slot_cell{i}.cartersian_coordinate_interested = task_temp.cartersian_coordinate_interested(test_start_index_lidar:test_end_index_lidar);
  

    % object detection in
    % test_slot_cell{i}.cartersian_coordinate_interested, record each
    % frame's e-vehicle x and y coordinates and calculate the speed of
    % e-vehicle later
    current_slot_lidar_frames = test_slot_cell{i}.cartersian_coordinate_interested; % current_slot_lidar_frames is a nx1 cell
    frames_number_in_current_slot = length(current_slot_lidar_frames);
    current_slot_trajectory_reconstruction=nan(frames_number_in_current_slot,2);%initialization
    unselected_clusters_record = cell(frames_number_in_current_slot,1);
    for j=1:frames_number_in_current_slot
        x_y_coordinatetemp = current_slot_lidar_frames{j}; % in nx2 format, x_y_coordinatetemp
        if isempty(x_y_coordinatetemp) || size(x_y_coordinatetemp,1) == 1 % if this frame has no point at all or just has one point, it doesn't make sense to continue the cluster method below, so we jusr skip this frame and left it as NaN as initialized
            continue;
        end       
        D = pdist2(x_y_coordinatetemp,x_y_coordinatetemp);%Pairwise distances between observations
        [idx, corepts] = dbscan(D,2,10,'Distance','precomputed'); % corepts means core points
        noise_indices = find(idx == -1); % when dbscan detect a cluster as noise, it automatically label that cluster as -1
        % noise_indices will be in one column format
        x_y_coordinatetemp(noise_indices',:)=[]; % delete noise points, i.e. label == -1 points
        idx(noise_indices')=[];
        % there is one possibility that there are no points at all in our
        % vision, so after the operation above, x_y_coordinatetemp ==[];
        % in this situation, we continue to the next loop and 
        %  leave current_slot_trajectory_reconstruction(j,:) == nan as
        %  initialization, this nan part we will fill in later.
        if isempty(x_y_coordinatetemp) % means that this frame only contains noise, so it doesn't make sense to continue,  so we left this frame be NaN as initialized
            continue;
        end
        
        % if x_y_coordinatetemp ~=[], then we do the following
        % now the current frame is without label==-1 noise
        % but we still don't know whether there are other noise clusters exist or not,
        % ideally, max(idx)== 1 because we have already cut out our test
        % field so our visual area in lidar should be clean and without any
        % other wierd clusters, but in reality you never know and it turns out to be seldom ideal.
        if max(idx) == 1
            % indicates that our vision is ideal, the only cluster is our
            % test participant.
            x_mean = mean(x_y_coordinatetemp(:,1));
            y_mean = mean(x_y_coordinatetemp(:,2));
            object_coordinate = [x_mean,y_mean];
            current_slot_trajectory_reconstruction(j,:)=object_coordinate;
        else
            % indicates that our vision is not ideal, there are more than
            % one cluster in our vision, so we need to decide which cluster
            % is our participant, we use convex hull to detect
            cluster_number = max(idx);
            area_save=nan(cluster_number,1); % initialize where we save all clusters convex hull area size
            for mm =1:cluster_number
                mm_indices = find(idx == mm); % mm_indices will be in one column format
                mm_cluster_x_y_coordinate = x_y_coordinatetemp(mm_indices',:); % nx2 format
                x= mm_cluster_x_y_coordinate(:,1);
                y= mm_cluster_x_y_coordinate(:,2);
                [~,V] = convhull(x,y);
                area_save(mm,1) = V;
            end
            % according to e-scooter techinical test, the convex hull of a
            % e-scooter is around 0.07532, we use this value to find which
            % cluster is the closest to this size and regard that cluster
            % as our participant
            expect_convexhull_size = 0.065; %!!!!!!!!!!will be modified later
            area_compare = abs(area_save - expect_convexhull_size); % the smallest element in area_compare represent the cluster we want
            % area_compare is in cluster_number x 1 format

            [~,cluster_wanted]=min(area_compare);
            cluster_wanted_indices = find(idx == cluster_wanted);
            cluster_wanted_x_y_coordinate = x_y_coordinatetemp(cluster_wanted_indices',:); % nx2 format
            x_mean = mean(cluster_wanted_x_y_coordinate(:,1));
            y_mean = mean(cluster_wanted_x_y_coordinate(:,2));
            object_coordinate = [x_mean,y_mean];
            current_slot_trajectory_reconstruction(j,:)=object_coordinate;
            unselected_record_temp = [];
            for c =1:cluster_number
                if c~=cluster_wanted
                    current_unselected_cluster_indices = find(idx == c);
                    current_unselected_cluster_x_y_coordinate = x_y_coordinatetemp(current_unselected_cluster_indices',:);
                    x_mean = mean(current_unselected_cluster_x_y_coordinate(:,1));
                    y_mean = mean(current_unselected_cluster_x_y_coordinate(:,2));
                    current_unselected_cluster_coordinate = [x_mean,y_mean];  
                    unselected_record_temp = [unselected_record_temp;current_unselected_cluster_coordinate];
                end 
            end
            unselected_clusters_record{j,1}=unselected_record_temp;

        end
                       
    end
    current_slot_trajectory_reconstruction=fillmissing(current_slot_trajectory_reconstruction,'linear',1);
    test_slot_cell{i}.trajectory_reconstruction = current_slot_trajectory_reconstruction;   
    test_slot_cell{i}.unselected_clusters_record = unselected_clusters_record;
    % test_slot_cell{i}.trajectory_reconstruction is in frames_number_in_current_slot x 2 format
    % differentiate current_slot_trajectory_reconstruction to get x and y
    % velocity
    % known condition: lidar sampling frequency is fixed, every timestamp
    % is 0.05s
    x_y_velocity_from_lidar = compute_speed_from_trajectory_reconstruction(current_slot_trajectory_reconstruction,test_slot_cell{i}.TimeStamp_lidar);
    test_slot_cell{i}.x_y_velocity_from_lidar = x_y_velocity_from_lidar; 

end
%========#####################==========savetest slot part=========######################===========
save('testslot_second_split.mat','test_slot_cell');
disp('Good! test slots have been second cut and saved')