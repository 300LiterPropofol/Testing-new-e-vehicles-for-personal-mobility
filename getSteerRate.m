function st_rate = getSteerRate(time, angle)
    
    st_rate = zeros(length(time), 1);
    
    st_rate(1, 1) = 0;
    
    for i = 2:length(time)
        st_rate(i) = (angle(i) - angle(i - 1)) / (time(i) - time(i - 1));
    end

end