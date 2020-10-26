function results = calculateResults(results, expState)
%CALCULATERESULTS Populates the averaged fields of the resultsStruct using
%the data in that and prints the key experiment results values. 

% The x,P,z fields of results must be cell arrays, one cell per full ekf
% run. Within each cell is a matrix of shape MxN where N is the number of
% steps and M is the number of states in that matrix (3 for x, 2 for z).
% For P, each cell contains a further array of N cells where each is a 3x3
% matrix.

%Construct the state estimate error matrix with each row being an EKF run and
%each column a timestep

numRuns = size(results.x,2);
numTimesteps = size(results.x{1},2);

assert(numRuns == expState.numRuns);
assert(numTimesteps == expState.numPoses);

for run = 1:numRuns
    for timestep = 1:numTimesteps
        results.xErrorMat(run,timestep) = results.x{run}(1,timestep) - results.targetPose{run}(1);
        results.yErrorMat(run,timestep) = results.x{run}(2,timestep) - results.targetPose{run}(2);
        results.zErrorMat(run,timestep) = results.x{run}(3,timestep) - results.targetPose{run}(3);
    end
end

%Calculate meanError
results.xErrorMean = mean(results.xErrorMat,1);
results.yErrorMean = mean(results.yErrorMat,1);
results.zErrorMean = mean(results.zErrorMat,1);

results.xErrorFinalMean = results.xErrorMean(end);
results.yErrorFinalMean = results.yErrorMean(end);
results.zErrorFinalMean = results.zErrorMean(end);

%Calculate errorStdDev
results.xErrorStd = std(results.xErrorMat);
results.yErrorStd = std(results.yErrorMat);
results.zErrorStd = std(results.zErrorMat);

results.xErrorFinalStd = std(results.xErrorMat(:,end));
results.yErrorFinalStd = std(results.yErrorMat(:,end));
results.zErrorFinalStd = std(results.zErrorMat(:,end));

%Final sigma trace and the std of this
for run = 1:numRuns
    finalTraces(run) = trace(results.P{run}{numTimesteps});
end
results.traceFinalMean = mean(finalTraces);
results.traceFinalStd = std(finalTraces);

%Calculate elementwise mean of sigma for each timestep
for timestep = 1:numTimesteps
    sigmaMean = zeros(3,3);
    for run = 1:numRuns
        sigmaMean = sigmaMean + results.P{run}{timestep}; %sigma mean over all runs
        traces(run,timestep) = trace(results.P{run}{timestep});
    end
    results.sigmaMean{timestep} = sigmaMean./numRuns;
    
    results.xCovMean(timestep) = results.sigmaMean{timestep}(1,1);
    results.yCovMean(timestep) = results.sigmaMean{timestep}(2,2);
    results.zCovMean(timestep) = results.sigmaMean{timestep}(3,3);
    
    results.traceStd(timestep) = std(traces(:,timestep));
    results.traceMean(timestep) = mean(traces(:,timestep));
end


fprintf("Mean final errors for %s experiment are: (%f, %f, %f)m \n", ... 
    expState.currExpName, results.xErrorFinalMean, results.yErrorFinalMean, ... 
    results.zErrorFinalMean);
fprintf("With std deviations: (%f, %f, %f)m \n", ... 
    results.xErrorFinalStd, results.yErrorFinalStd, ... 
    results.zErrorFinalStd);
fprintf("And final sigma trace: %f \n", results.traceFinalMean);
fprintf("With std deviation: %f \n", results.traceFinalStd);

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
    errorbar(1:numTimesteps,results.traceMean,results.traceStd,'r');
    xlabel('Filter Timestep');
    grid on
    
    %Show the sigma component plots over time
    figure();
    hold on
    title(strcat(expState.currExpName, ' State Covariance Components'));
    plot(1:numTimesteps,results.xCovMean,'r');
    plot(1:numTimesteps,results.yCovMean,'g');
    plot(1:numTimesteps,results.zCovMean,'b');
    xlabel('Filter Timestep');
    legend('X','Y','Z');
    grid on
end

end