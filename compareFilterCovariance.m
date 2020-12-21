%Generate a plot of the mean filter variance in x,y,z for each method
clear all
close all

load('Exp1Results.mat');
load('Exp2Results.mat');
load('Exp3Results.mat');

numSteps = 1:size(straightResults.xErrorMat,2);

%To make pretty colours
N=4;
C = linspecer(N);

offset = 0.0;

%X Component
fig = figure();
title('Mean Filter State Estimate X Variance');
hold on
lw = 1.2;
eb = plot(numSteps,straightResults.xCovMean','color',C(1,:));
eb.LineWidth = lw;
eb = plot(numSteps+offset,diagonalResults.xCovMean','color',C(2,:));
eb.LineWidth = lw;
eb = plot(numSteps+2*offset,FVIofflineGraspResults.xCovMean','color',C(3,:));
eb.LineWidth = lw;
eb = plot(numSteps+3*offset,FVIonlineGraspResults.xCovMean','color',C(4,:));
eb.LineWidth = lw;
plot(numSteps,zeros(size(numSteps,2),1),'k');
xlabel('Filter Timestep');
legend('Straight Path', 'Diagonal Path', 'Offline RVI Constrained', 'Online RVI Constrained');
grid minor
ylabel('Variance (m^2)');

%Y Component
fig = figure();
title('Mean Filter State Estimate Y Variance');
hold on
lw = 1.2;
eb = plot(numSteps,straightResults.yCovMean','color',C(1,:));
eb.LineWidth = lw;
eb = plot(numSteps+offset,diagonalResults.yCovMean','color',C(2,:));
eb.LineWidth = lw;
eb = plot(numSteps+2*offset,FVIofflineGraspResults.yCovMean','color',C(3,:));
eb.LineWidth = lw;
eb = plot(numSteps+3*offset,FVIonlineGraspResults.yCovMean','color',C(4,:));
eb.LineWidth = lw;
plot(numSteps,zeros(size(numSteps,2),1),'k');
xlabel('Filter Timestep');
grid minor

%Z Component
fig = figure();
title('Mean Filter State Estimate Z Variance');
hold on
lw = 1.2;
eb = plot(numSteps,straightResults.zCovMean','color',C(1,:));
eb.LineWidth = lw;
eb = plot(numSteps+offset,diagonalResults.zCovMean','color',C(2,:));
eb.LineWidth = lw;
eb = plot(numSteps+2*offset,FVIofflineGraspResults.zCovMean','color',C(3,:));
eb.LineWidth = lw;
eb = plot(numSteps+3*offset,FVIonlineGraspResults.zCovMean','color',C(4,:));
eb.LineWidth = lw;
plot(numSteps,zeros(size(numSteps,2),1),'k');
xlabel('Filter Timestep');
grid minor