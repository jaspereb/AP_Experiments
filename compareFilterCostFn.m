%Generate a plot of the mean filter weighted trace cost function for each step
clear all
close all

load('Exp1Results.mat');
load('Exp2Results.mat');
load('Exp3Results.mat');

numSteps = 1:size(straightResults.xErrorMat,2);

%To make pretty colours
N=4;
C = linspecer(N);

offset = 0.08;

fig = figure();
title('Mean Filter Cost Function');
hold on
lw = 1.2;
eb = errorbar(numSteps,straightResults.costMean,straightResults.costStd,'color',C(1,:));
eb.LineWidth = lw;
capErrorBars(eb);
eb = errorbar(numSteps+offset,diagonalResults.costMean,diagonalResults.costStd,'color',C(2,:));
eb.LineWidth = lw;
capErrorBars(eb);
eb = errorbar(numSteps+2*offset,FVIofflineGraspResults.costMean,FVIofflineGraspResults.costStd,'color',C(3,:));
eb.LineWidth = lw;
capErrorBars(eb);
eb = errorbar(numSteps+3*offset,FVIonlineGraspResults.costMean,FVIonlineGraspResults.costStd,'color',C(4,:));
eb.LineWidth = lw;
capErrorBars(eb);
plot(numSteps,zeros(size(numSteps,2),1),'k');
xlabel('Filter Timestep');
legend('Straight Path', 'Diagonal Path', 'Offline RVI Constrained', 'Online RVI Constrained');
grid minor
ylabel('Cost');

