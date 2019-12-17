function [slalom_start_index, slalom_end_index] = getSlalomIndices(brake_start_index, time, st_rate, gyro_x, t_over_thsd, st_rate_thsd, gyro_x_thsd, max_duration)
    
    st_rate = abs(st_rate);
    gyro_x = abs(gyro_x);
    
    slalom_start_index = 1;
    slalom_end_index = 1;
    
    t_over = 0;

    for i = brake_start_index:-1:1
        if (st_rate(i) > st_rate_thsd || gyro_x(i) > gyro_x_thsd) && slalom_end_index == 1
            slalom_end_index = i;
        elseif (st_rate(i) <= st_rate_thsd && gyro_x(i) < gyro_x_thsd) && slalom_end_index > 1 && slalom_start_index == 1
            slalom_start_index = i;
        elseif (st_rate(i) > st_rate_thsd || gyro_x(i) > gyro_x_thsd) && slalom_end_index > 1
            slalom_start_index = 1;
            t_over = 0;
        end
        
        if time(slalom_end_index) - time(i) > max_duration
            slalom_start_index = i;
            break;
        end
        
        if t_over > t_over_thsd && slalom_end_index > 1
            break;
        else
            t_over = time(slalom_start_index) - time(i);
        end 
    end
    
end