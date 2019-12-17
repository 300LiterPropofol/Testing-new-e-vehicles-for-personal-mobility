function r2 = calcR2Val(polyfit, x_val, y_val)

    yfit = polyval(polyfit, x_val);
    yresid = y_val - yfit;
    
    SSresid = sum(yresid .^ 2);
    SStotal = (length(y_val) - 1) * var(y_val);
    
    r2 = 1 - SSresid/SStotal;

end