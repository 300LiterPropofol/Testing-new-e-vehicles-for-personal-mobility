function x_y_velocity_from_lidar = compute_speed_from_trajectory_reconstruction(current_slot_trajectory_reconstruction,TimeStamp_lidar)
x_coordinates = current_slot_trajectory_reconstruction(:,1);
y_coordinates = current_slot_trajectory_reconstruction(:,2);
x_speed_temp = nan((length(TimeStamp_lidar)-1),1);
y_speed_temp = nan((length(TimeStamp_lidar)-1),1);
for i =1:length(TimeStamp_lidar)-1
    x_speed_temp(i)=(x_coordinates(i+1)-x_coordinates(i))/(TimeStamp_lidar(i+1)-TimeStamp_lidar(i));
    y_speed_temp(i)=(y_coordinates(i+1)-y_coordinates(i))/(TimeStamp_lidar(i+1)-TimeStamp_lidar(i));
end
x_velocity_from_lidar=[0;x_speed_temp(1:end-1);0];
y_velocity_from_lidar=[0;y_speed_temp(1:end-1);0];
x_y_velocity_from_lidar = [x_velocity_from_lidar,y_velocity_from_lidar];