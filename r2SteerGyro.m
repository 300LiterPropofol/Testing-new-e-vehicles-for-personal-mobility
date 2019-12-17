function value = r2SteerGyro(st_rate, gyro_x, p_rate, segment_start, segment_end)
    if segment_start < segment_end
        value = calcR2Val(p_rate, st_rate(segment_start:segment_end), gyro_x(segment_start:segment_end));
    else
        value = NaN;
    end
end