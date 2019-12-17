function [dec_start_index, brake_start_index] = getBrakeIndex(end_brake_index, speed_thsd, time, acc_x, speed, brake_t_over_thsd)

    acc_x = abs(acc_x);

    brake_start_index = 1;
    dec_start_index = 1;
        
    
    for i = end_brake_index:-1:1
        if acc_x(i) < 1
            t_over = time(end_brake_index) - time(i);

            if t_over > brake_t_over_thsd
                break;
            end
        else
            brake_start_index = i;
        end
    end
    
    for i = brake_start_index:-1:1
        if speed(i) > speed_thsd
            dec_start_index = i;
            break;
        end
    end

end