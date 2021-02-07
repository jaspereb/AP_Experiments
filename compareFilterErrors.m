%Generate a plot of the mean error magnitude for each method
clear all
close all
addpath('Utils/linspecer');

load('Exp1Results.mat');
load('Exp2Results.mat');
load('Exp3Results.mat');

numSteps = 1:size(straightResults.xErrorMat,2);

%To make pretty colours
N=4;
C = linspecer(N);
    
offset = 0.08;

%X Component
fig = figure();
title('Mean Filter X Error');
hold on
lw = 1.2;
eb = errorbar(numSteps,straightResults.xErrorMean,straightResults.xErrorStd,'color',C(1,:));
eb.LineWidth = lw;
eb = errorbar(numSteps+offset,diagonalResults.xErrorMean,diagonalResults.xErrorStd,'color',C(2,:));
eb.LineWidth = lw;
eb = errorbar(numSteps+2*offset,FVIofflineGraspResults.xErrorMean,FVIofflineGraspResults.xErrorStd,'color',C(3,:));
eb.LineWidth = lw;
eb = errorbar(numSteps+3*offset,FVIonlineGraspResults.xErrorMean,FVIonlineGraspResults.xErrorStd,'color',C(4,:));
eb.LineWidth = lw;
plot(numSteps,zeros(size(numSteps,2),1),'k');
xlabel('Filter Timestep');
legend('Straight Path', 'Diagonal Path', 'Offline RVI Constrained', 'Online RVI Constrained');
grid minor
ylabel('Error (m)');

%Y Component
fig = figure();
title('Mean Filter Y Error');
hold on
lw = 1.2;
eb = errorbar(numSteps,straightResults.yErrorMean,straightResults.yErrorStd,'color',C(1,:));
eb.LineWidth = lw;
eb = errorbar(numSteps+offset,diagonalResults.yErrorMean,diagonalResults.yErrorStd,'color',C(2,:));
eb.LineWidth = lw;
eb = errorbar(numSteps+2*offset,FVIofflineGraspResults.yErrorMean,FVIofflineGraspResults.yErrorStd,'color',C(3,:));
eb.LineWidth = lw;
eb = errorbar(numSteps+3*offset,FVIonlineGraspResults.yErrorMean,FVIonlineGraspResults.yErrorStd,'color',C(4,:));
eb.LineWidth = lw;
plot(numSteps,zeros(size(numSteps,2),1),'k');
xlabel('Filter Timestep');
% legend('Straight Path', 'Diagonal Path', 'Offline RVI Constrained', 'Online RVI Constrained');
grid minor

%Z Component
fig = figure();
title('Mean Filter Z Error');
hold on
lw = 1.2;
eb = errorbar(numSteps,straightResults.zErrorMean,straightResults.zErrorStd,'color',C(1,:));
eb.LineWidth = lw;
eb = errorbar(numSteps+offset,diagonalResults.zErrorMean,diagonalResults.zErrorStd,'color',C(2,:));
eb.LineWidth = lw;
eb = errorbar(numSteps+2*offset,FVIofflineGraspResults.zErrorMean,FVIofflineGraspResults.zErrorStd,'color',C(3,:));
eb.LineWidth = lw;
eb = errorbar(numSteps+3*offset,FVIonlineGraspResults.zErrorMean,FVIonlineGraspResults.zErrorStd,'color',C(4,:));
eb.LineWidth = lw;
plot(numSteps,zeros(size(numSteps,2),1),'k');
xlabel('Filter Timestep');
% legend('Straight Path', 'Diagonal Path', 'Offline RVI Constrained', 'Online RVI Constrained');
grid minor

%Full 6 item version
% fig = figure();
% title('Mean Filter X Error');
% hold on
% lw = 1.2;
% eb = errorbar(numSteps,straightResults.xErrorMean,straightResults.xErrorStd,'r');
% eb.LineWidth = lw;
% eb = errorbar(numSteps+0.03,diagonalResults.xErrorMean,diagonalResults.xErrorStd,'g');
% eb.LineWidth = lw;
% eb = errorbar(numSteps+0.06,FVIofflineGraspResults.xErrorMean,FVIofflineGraspResults.xErrorStd,'b');
% eb.LineWidth = lw;
% eb = errorbar(numSteps+0.09,FVIofflineViewResults.xErrorMean,FVIofflineViewResults.xErrorStd,'m');
% eb.LineWidth = lw;
% eb = errorbar(numSteps+0.12,FVIonlineGraspResults.xErrorMean,FVIonlineGraspResults.xErrorStd,'y');
% eb.LineWidth = lw;
% eb = errorbar(numSteps+0.15,FVIonlineViewResults.xErrorMean,FVIonlineViewResults.xErrorStd,'c');
% eb.LineWidth = lw;
% plot(numSteps,zeros(size(numSteps,2),1),'k');
% xlabel('Filter Timestep');
% legend('Straight Path', 'Diagonal Path', 'Offline RVI Constrained', 'Offline RVI Unconstrained', 'Online RVI Constrained', 'Online RVI Unconstrained');
% grid on