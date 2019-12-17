function value = delaySteerGyro(time, st_rate, gyro_x, segment_start, segment_end)
    if segment_start < segment_end
        value = finddelay(gyro_x(segment_start:segment_end), st_rate(segment_start:segment_end)) * mean(diff(time(segment_start:segment_end)));
    else
        value = NaN;
    end
end