%Load the 3 results files and print out the table of numbers
clear all
close all

load('Exp1Results.mat');
load('Exp2Results.mat');
load('Exp3Results.mat');

%Straight Path
err = [straightResults.xErrorFinalMean,straightResults.yErrorFinalMean,straightResults.zErrorFinalMean];
cov = [straightResults.xErrorFinalStd,straightResults.yErrorFinalStd,straightResults.zErrorFinalStd];
cost = straightResults.costFinalMean;

fprintf('Straight Path: \n');
fprintf('Errors; x: %f, y: %f, z: %f \n', err(1), err(2), err(3));
fprintf('Covariance; x: %f, y: %f, z: %f \n', cov(1), cov(2), cov(3));
fprintf('Cost; %f \n', cost);

%Diagonal Path
err = [diagonalResults.xErrorFinalMean,diagonalResults.yErrorFinalMean,diagonalResults.zErrorFinalMean];
cov = [diagonalResults.xErrorFinalStd,diagonalResults.yErrorFinalStd,diagonalResults.zErrorFinalStd];
cost = diagonalResults.costFinalMean;

fprintf('Diagonal Path: \n');
fprintf('Errors; x: %f, y: %f, z: %f \n', err(1), err(2), err(3));
fprintf('Covariance; x: %f, y: %f, z: %f \n', cov(1), cov(2), cov(3));
fprintf('Cost; %f \n', cost);

%Offline Constrained
err = [FVIofflineGraspResults.xErrorFinalMean,FVIofflineGraspResults.yErrorFinalMean,FVIofflineGraspResults.zErrorFinalMean];
cov = [FVIofflineGraspResults.xErrorFinalStd,FVIofflineGraspResults.yErrorFinalStd,FVIofflineGraspResults.zErrorFinalStd];
cost = FVIofflineGraspResults.costFinalMean;

fprintf('Offline Constrained: \n');
fprintf('Errors; x: %f, y: %f, z: %f \n', err(1), err(2), err(3));
fprintf('Covariance; x: %f, y: %f, z: %f \n', cov(1), cov(2), cov(3));
fprintf('Cost; %f \n', cost);
fprintf('std; %f \n', FVIofflineGraspResults.costFinalStd);


%Offline Unconstrained
err = [FVIofflineViewResults.xErrorFinalMean,FVIofflineViewResults.yErrorFinalMean,FVIofflineViewResults.zErrorFinalMean];
cov = [FVIofflineViewResults.xErrorFinalStd,FVIofflineViewResults.yErrorFinalStd,FVIofflineViewResults.zErrorFinalStd];
cost = FVIofflineViewResults.costFinalMean;

fprintf('Offline Unconstrained: \n');
fprintf('Errors; x: %f, y: %f, z: %f \n', err(1), err(2), err(3));
fprintf('Covariance; x: %f, y: %f, z: %f \n', cov(1), cov(2), cov(3));
fprintf('Cost; %f \n', cost);
fprintf('std; %f \n', FVIofflineViewResults.costFinalStd);

%Online Constrained
err = [FVIonlineGraspResults.xErrorFinalMean,FVIonlineGraspResults.yErrorFinalMean,FVIonlineGraspResults.zErrorFinalMean];
cov = [FVIonlineGraspResults.xErrorFinalStd,FVIonlineGraspResults.yErrorFinalStd,FVIonlineGraspResults.zErrorFinalStd];
cost = FVIonlineGraspResults.costFinalMean;

fprintf('Online Constrained: \n');
fprintf('Errors; x: %f, y: %f, z: %f \n', err(1), err(2), err(3));
fprintf('Covariance; x: %f, y: %f, z: %f \n', cov(1), cov(2), cov(3));
fprintf('Cost; %f \n', cost);

%Online Unconstrained
err = [FVIonlineViewResults.xErrorFinalMean,FVIonlineViewResults.yErrorFinalMean,FVIonlineViewResults.zErrorFinalMean];
cov = [FVIonlineViewResults.xErrorFinalStd,FVIonlineViewResults.yErrorFinalStd,FVIonlineViewResults.zErrorFinalStd];
cost = FVIonlineViewResults.costFinalMean;

fprintf('Online Unconstrained: \n');
fprintf('Errors; x: %f, y: %f, z: %f \n', err(1), err(2), err(3));
fprintf('Covariance; x: %f, y: %f, z: %f \n', cov(1), cov(2), cov(3));
fprintf('Cost; %f \n', cost);


%Print Timing Table
fprintf('Straight Time: %f \n', straightTime);
fprintf('Diagonal Time: %f \n', diagonalTime);
fprintf('Offline RVI Time: %f \n', offlineRVITime);
fprintf('Online RVI Time: %f \n', onlineRVITime);


