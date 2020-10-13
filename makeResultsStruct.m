function resultsStruct = makeResultsStruct(expState)
% Makes a struct to hold experiment results that can be used with
% calculateResults(resultsStruct,expState)

% N is the number of steps in each EKF run, M is the number of EKF runs

resultsStruct.x = {}; %A cell array of each EKF run result (each cell is 3xN)
resultsStruct.P = {}; %A cell array of each EKF run result (each cell is a cell containing 3x3)
resultsStruct.z = {}; %A cell array of each EKF run result (each cell is 3xN)

resultsStruct.meanError = [0;0;0];
resultsStruct.errorStdDev = [0;0;0];
resultsStruct.finalTrace = 0; %Final tr(P) value

%A cell array of the mean sigma for each N timestep over all M EKF runs
resultsStruct.meanSig = {}; 

%A copy of the experiment settings used to generate this result
resultsStruct.expState = expState;

end