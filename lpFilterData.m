function output = lpFilterData(data, fcut, fs)

  [b, a] = butter(1, fcut * 2 / fs);
  output = filtfilt(b, a, data);
  
end