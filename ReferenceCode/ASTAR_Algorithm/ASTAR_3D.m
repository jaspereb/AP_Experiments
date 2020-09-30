function OptimalPath=ASTAR_3D(StartX,StartY,StartZ,MAP,GoalRegister,Connecting_Distance)
%Version 1.0
% By Einar Ueland 2nd of May, 2016

%FINDING ASTAR PATH IN AN OCCUPANCY GRID

useSimpleNeighbours = false;
%Note, using simple neighbours is not actually faster because the heuristic
%is less informative 

%nNeighboor=3;
% Preallocation of Matrices
[Height,Width,Depth]=size(MAP); %Height and width of matrix
GScore=zeros(Height,Width,Depth);           %Matrix keeping track of G-scores
FScore=single(inf(Height,Width,Depth));     %Matrix keeping track of F-scores (only open list)
Hn=single(zeros(Height,Width,Depth));       %Heuristic matrix
OpenMAT=int8(zeros(Height,Width,Depth));    %Matrix keeping of open grid cells
ClosedMAT=int8(zeros(Height,Width,Depth));  %Matrix keeping track of closed grid cells
ClosedMAT(MAP==1)=1;                  %Adding object-cells to closed matrix
ParentX=int16(zeros(Height,Width,Depth));   %Matrix keeping track of X position of parent
ParentY=int16(zeros(Height,Width,Depth));   %Matrix keeping track of Y position of parent
ParentZ=int16(zeros(Height,Width,Depth));

% %%% Setting up matrices representing neighboors to be investigated
% NeighboorCheck=ones(2*Connecting_Distance+1);
% Dummy=2*Connecting_Distance+2;
% Mid=Connecting_Distance+1;
% for i=1:Connecting_Distance-1
% NeighboorCheck(i,i)=0;
% NeighboorCheck(Dummy-i,i)=0;
% NeighboorCheck(i,Dummy-i)=0;
% NeighboorCheck(Dummy-i,Dummy-i)=0;
% NeighboorCheck(Mid,i)=0;
% NeighboorCheck(Mid,Dummy-i)=0;
% NeighboorCheck(i,Mid)=0;
% NeighboorCheck(Dummy-i,Mid)=0;
% end
% NeighboorCheck(Mid,Mid)=0;
%
% [row, col]=find(NeighboorCheck==1);
% Neighboors=[row col]-(Connecting_Distance+1);
% N_Neighboors=size(col,1);
% %%% End of setting up matrices representing neighboors to be investigated

if(useSimpleNeighbours)
    %Simple neighbours
    Neighboors = [];
    actions = [-Connecting_Distance,0,0];
    Neighboors = [Neighboors; actions];
    actions = [Connecting_Distance,0,0];
    Neighboors = [Neighboors; actions];
    actions = [0,-Connecting_Distance,0];
    Neighboors = [Neighboors; actions];
    actions = [0,Connecting_Distance,0];
    Neighboors = [Neighboors; actions];
    actions = [0,0,-Connecting_Distance];
    Neighboors = [Neighboors; actions];
    actions = [0,0,Connecting_Distance];
    Neighboors = [Neighboors; actions];
    N_Neighboors=size(Neighboors,1);
else
    %Fully connected neighbours
    Neighboors = [];
    for i=-Connecting_Distance:1:Connecting_Distance
        for j=-Connecting_Distance:1:Connecting_Distance
            for k=-Connecting_Distance:1:Connecting_Distance
                Neighboors(end+1,:) = [i,j,k];
            end
        end
    end
    idx = find(ismember(Neighboors,[0,0,0],'rows'));
    Neighboors(idx,:) = [];
    N_Neighboors=size(Neighboors,1);
end

%%%%%%%%% Creating Heuristic-matrix based on distance to nearest  goal node
% [col, row, dep]=find(GoalRegister==1);
[col, row, dep] = ind2sub(size(GoalRegister),find(GoalRegister == 1));
RegisteredGoals=[row col dep];
Nodesfound=size(RegisteredGoals,1);

for k=1:size(GoalRegister,1)
    for j=1:size(GoalRegister,2)
        for l=1:size(GoalRegister,3)
            if MAP(k,j,l)==0
                if(useSimpleNeighbours)
                    Mat=RegisteredGoals-(repmat([j k l],(Nodesfound),1)); %If using simple neighbours
                    Distance=(min(sum(abs(Mat),2)));
                else
                    Mat=RegisteredGoals-(repmat([j k l],(Nodesfound),1)); %If using diagonal neighbours
                    Distance=(min(sqrt(sum(abs(Mat).^2,2))));
                end
                Hn(k,j,l)=Distance;
            end
        end
    end
end
%End of creating Heuristic-matrix.

%Note: If Hn values is set to zero the method will reduce to the Dijkstras method.

%Initializign start node with FValue and opening first node.
FScore(StartY,StartX,StartZ)=Hn(StartY,StartX,StartZ);
OpenMAT(StartY,StartX,StartZ)=1;




while 1==1 %Code will break when path found or when no path exist
    
    closed = (ClosedMAT==1);
    closed = sum(closed(:));
    closed = 100*(closed/numel(MAP));
    fprintf(' %f%% of nodes closed\n', closed);
    
    [MINopenFSCORE,minPos]=min(FScore(:));
    if MINopenFSCORE==inf
        %Failuere!
        OptimalPath=[inf];
        RECONSTRUCTPATH=0;
        break
    end
    [CurrentY,CurrentX,CurrentZ]=ind2sub(size(FScore),minPos(1));
    
    CurrentY=CurrentY(1);
    CurrentX=CurrentX(1);
    CurrentZ=CurrentZ(1);
    fprintf('Current position is %d,%d,%d \n', CurrentX, CurrentY, CurrentZ);
    
    if GoalRegister(CurrentY,CurrentX,CurrentZ)==1
        %GOAL!!
        RECONSTRUCTPATH=1;
        break
    end
    
    %Remobing node from OpenList to ClosedList
    OpenMAT(CurrentY,CurrentX,CurrentZ)=0;
    FScore(CurrentY,CurrentX,CurrentZ)=inf;
    ClosedMAT(CurrentY,CurrentX,CurrentZ)=1;
    for p=1:N_Neighboors
        i=Neighboors(p,1); %Y
        j=Neighboors(p,2); %X
        k=Neighboors(p,3); %Z
        if CurrentY+i<1||CurrentY+i>Height||CurrentX+j<1||CurrentX+j>Width||CurrentZ+k<1||CurrentZ+k>Depth
            continue
        end
        Flag=1;
        if(ClosedMAT(CurrentY+i,CurrentX+j,CurrentZ+k)==0) %Neiboor is open;
            if (abs(i)>1||abs(j)>1||abs(k)>1)
                % Need to check that the path does not pass an object
                JumpCells=2*max([abs(i),abs(j),abs(k)])-1;
                for K=1:JumpCells;
                    YPOS=round(K*i/JumpCells);
                    XPOS=round(K*j/JumpCells);
                    ZPOS=round(K*k/JumpCells);
                    
                    if (MAP(CurrentY+YPOS,CurrentX+XPOS,CurrentZ+ZPOS)==1)
                        Flag=0;
                    end
                end
            end
            %End of  checking that the path does not pass an object
            
            if Flag==1
                tentative_gScore = GScore(CurrentY,CurrentX,CurrentZ) + sqrt(i^2+j^2+k^2);
                if OpenMAT(CurrentY+i,CurrentX+j,CurrentZ+k)==0
                    OpenMAT(CurrentY+i,CurrentX+j,CurrentZ+k)=1;
                elseif tentative_gScore >= GScore(CurrentY+i,CurrentX+j,CurrentZ+k)
                    continue
                end
                ParentX(CurrentY+i,CurrentX+j,CurrentZ+k)=CurrentX; %ORDER MAY BE WRONG
                ParentY(CurrentY+i,CurrentX+j,CurrentZ+k)=CurrentY;
                ParentZ(CurrentY+i,CurrentX+j,CurrentZ+k)=CurrentZ;
                GScore(CurrentY+i,CurrentX+j,CurrentZ+k)=tentative_gScore;
                FScore(CurrentY+i,CurrentX+j,CurrentZ+k)= tentative_gScore+Hn(CurrentY+i,CurrentX+j,CurrentZ+k);
            end
        end
    end
end

k=2;
if RECONSTRUCTPATH
    OptimalPath(1,:)=[CurrentY CurrentX CurrentZ];
    while RECONSTRUCTPATH
        CurrentZDummy=ParentZ(CurrentY,CurrentX,CurrentZ);
        CurrentXDummy=ParentX(CurrentY,CurrentX,CurrentZ);
        CurrentY=ParentY(CurrentY,CurrentX,CurrentZ);
        CurrentX=CurrentXDummy;
        CurrentZ=CurrentZDummy;
        
        OptimalPath(k,:)=[CurrentY CurrentX CurrentZ];
        k=k+1;
        if (((CurrentX== StartX)) &&(CurrentY==StartY) &&(CurrentZ==StartZ))
            break
        end
    end
end


end



%{
line 93 -106 is using brute force to check if the path passes an object. For large connecting_distances
this uses alot of CPU time. If you have any more efficient way to add this effect in the code, pleas contact me and
I will edit the code.
%}
