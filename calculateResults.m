function results = calculateResults(results, expState)
%CALCULATERESULTS Populates the averaged fields of the resultsStruct using
%the data in that and prints the key experiment results values. 

%Construct the state estimate error matrix with each row being an EKF run and
%each column a timestep
for row = 1:size(results.x,2)
    for col = 1:size(results.x{1},2)
        results.xErrorMat(row,col) = results.targetPose(1) - results.x{row}(1,col);
        results.yErrorMat(row,col) = results.targetPose(2) - results.x{row}(2,col);
        results.zErrorMat(row,col) = results.targetPose(3) - results.x{row}(3,col);
    end
end

%Calculate meanError
results.xErrorMean = mean(results.xErrorMat,1);
results.yErrorMean = mean(results.yErrorMat,1);
results.zErrorMean = mean(results.zErrorMat,1);

results.xErrorSingleMean = mean(results.xErrorMean);
results.yErrorSingleMean = mean(results.yErrorMean);
results.zErrorSingleMean = mean(results.zErrorMean);

%Calculate errorStdDev
results.xErrorStd = std(results.xErrorMat);
results.yErrorStd = std(results.yErrorMat);
results.zErrorStd = std(results.zErrorMat);

results.xErrorSingleStd = std(results.xErrorMat(:));
results.yErrorSingleStd = std(results.yErrorMat(:));
results.zErrorSingleStd = std(results.zErrorMat(:));

%Calculate elementwise mean of sigma for each timestep
for step = 1:size(results.x{1},2)
    clearvars sigmaMean traces
    sigmaMean{step} = zeros(3,3);
    for run = 1:size(results.P,2)
        sigmaMean{step} = sigmaMean{step} + results.P{run}{step};
        traces(run) = trace(results.P{run}{end});
    end
    results.sigmaMean{step} = sigmaMean{step}./size(results.P,2);
    
    results.xCovMean(step) = results.sigmaMean{step}(1,1);
    results.yCovMean(step) = results.sigmaMean{step}(2,2);
    results.zCovMean(step) = results.sigmaMean{step}(3,3);
    
    results.traceMean(step) = trace(results.sigmaMean{step});
    results.traceStd(step) = std(traces);
end
results.traceSingleMean = mean(results.traceMean);

fprintf("Mean final errors for %s experiment are: (%f, %f, %f)m \n", ... 
    expState.currExpName, results.xErrorSingleMean, results.yErrorSingleMean, ... 
    results.zErrorSingleMean);
fprintf("With std deviations: (%f, %f, %f)m \n", ... 
    results.xErrorSingleStd, results.yErrorSingleStd, ... 
    results.zErrorSingleStd);
fprintf("And final sigma trace: %f \n", results.traceSingleMean);

if(expState.plotResults)
    %Show the mean error and std plots over time
    figure();
    hold on
    title(strcat(expState.currExpName, ' Mean Error'));
    errorbar(1:size(results.xErrorMean,2),results.xErrorMean,results.xErrorStd,'r');
    errorbar(1:size(results.yErrorMean,2),results.yErrorMean,results.yErrorStd,'g');
    errorbar(1:size(results.zErrorMean,2),results.zErrorMean,results.zErrorStd,'b');
    xlabel('Filter Timestep');
    legend('X','Y','Z');
    grid on
    
    %Show the mean tr(sigma) plots over time
    figure();
    hold on
    title(strcat(expState.currExpName, ' Sigma Trace'));
    errorbar(1:size(results.traceMean,2),results.traceMean,results.traceStd,'r');
    xlabel('Filter Timestep');
    grid on
    
    %Show the sigma component plots over time
    figure();
    hold on
    title(strcat(expState.currExpName, ' State Covariance Components'));
    plot(1:size(results.xCovMean,2),results.xCovMean,'r');
    plot(1:size(results.yCovMean,2),results.yCovMean,'g');
    plot(1:size(results.zCovMean,2),results.zCovMean,'b');
    xlabel('Filter Timestep');
    legend('X','Y','Z');
    grid on
end

end