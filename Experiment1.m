%Experiment 1 compares the estimates from 1000 runs of the straight path
%and 1000 of the diagonal path

load('StartState.mat');

% Straight Path Experiment
straightPoses = getPosesStraight(cameraPose,expState);
expState.currExpName = 'Straight Path';
if(expState.showFigs)
    fig_straight = figure();
    plotState(straightPoses, expState, fig_straight);
end
[x_straight,P_straight,z_straight] = runEKF(straightPoses,expState,functH,x,Q,R,A,K,C,P);

% Diagonal Path Experiment
diagonalPoses = getPosesDiagonal(cameraPose,expState);
expState.currExpName = 'Diagonal Path';
if(expState.showFigs)
    fig_diag = figure();
    plotState(diagonalPoses, expState, fig_diag);
end
[x_diag,P_diag,z_diag] = runEKF(diagonalPoses,expState,functH,x,Q,R,A,K,C,P);