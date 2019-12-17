% test_temp = test_slot_cell{1,1};
% time_imu = test_temp.TimeStamp_imu;
% acc_x = test_temp.LinearAcceleration_x_filtered;
% gyro_x = test_temp.AngularVelocity_x_filtered;
% 
% x_y_speed = test_temp.x_y_velocity_from_lidar;
% x_speed = x_y_speed(:,1);
% [~, ~] = spectrumanalysis(x_speed, 1/0.05);

% for k = 1:16
% 	plot(fft(eye(k+16)))
% 	axis([-1 1 -1 1])
% 	M(k) = getframe;
% end
% figure
% movie(M,5)

A = [9 0 -7 5 3 8 -10 4 2];
[B,I] = sort(A);