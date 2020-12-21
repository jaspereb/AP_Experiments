function capErrorBars(eb)
%CAPERRORBARS Takes and error bar handle and truncates at zero.

for i = 1:size(eb.YNegativeDelta,2)
    if(eb.YData(i) - eb.YNegativeDelta(i) < 0)
        if(eb.YData(i) < 0)
            print("ERROR: Truncating error bar with negative mean");
            return
        end
        
        eb.YNegativeDelta(i) = -eb.YData(i);
    end
end

end

