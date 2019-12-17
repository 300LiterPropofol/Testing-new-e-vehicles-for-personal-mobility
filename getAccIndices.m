function [start_acc_index, const_speed_start, end_brake_index] = getAccIndices(time, speed, min_speed_thsd, req_speed, min_t_over_thsd)

    start_acc_index = 1;
    end_brake_index = 1;
    const_speed_start = 1;
    
    t_over = 0;

    for i = 1:length(time)
        if speed(i) > min_speed_thsd
            if start_acc_index == 1
                start_acc_index = i;
            end

            if speed(i) > req_speed && const_speed_start == 1
                const_speed_start = i;
            end

            t_over = time(i) - time(start_acc_index);
        else
            if t_over > min_t_over_thsd
                end_brake_index = i;
                break;
            else
                start_acc_index = 1;
            end
        end
    end

end