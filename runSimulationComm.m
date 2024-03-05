% Execution of a single simulation with an specific parameters. The environment especification should be included in 
% the directory environments.
%Parameters: 
%    - Nt: number of task 
%    - Nrobots: number of robots.
%    - NstepsSim: number of steps of the simulation
%    - nEx: number of the environment to test
%    - maxDistComm: MAX_RANGE
%    - wNrobot: omega_w

function [distanceR, timesVisitedTask, allTasksEndT]=runSimulationComm(Nt, Nrobots, NstepsSim, nEx, maxDistComm, wNrobot)

  
  fname=sprintf('environments/EnvironmentSw_%d',nEx); %open the file with the environment
  load(fname, 'posObjE'); 
  posObj=posObjE(1:Nt,:);
  
  %Length=Nrobots-->For each one selects an initial task
  %currTaskRobot=randi([1 Nt], [1 Nrobots]); %Random selection of the initial task for each robot
  currTaskRobot=repmat(1,1,Nrobots);
  
  newCurrTask=zeros(1,Nrobots);
  timesVisitedTask=zeros(1,Nt); % Numer of times that tis task has been visited.
  distanceR=zeros(1,Nrobots);
  
  
  %init timesVisited
  for i=1:Nrobots
      timesVisitedTask(currTaskRobot(i)) = timesVisitedTask(currTaskRobot(i)) + 1;
  end
  
  %See TH value in getConvergence.m
  dmax=2*283*sqrt(2);
  multDist=4;
  TH=multDist/dmax;
  %maxDistComm=dmax/10;
  
  allTasksEndT = (-1); %Steps required to visit all the tasks (-1) the condition is not reached  
  for nS=1:NstepsSim
      [PF, Pro] = createTransMatrixComm(posObj, Nt, 1, 2, TH, currTaskRobot, maxDistComm, wNrobot*dmax/Nrobots);
      for iRobot=1:Nrobots %abans Par
          nextTaskAux = nextTaskRobotComm(PF, currTaskRobot(iRobot)); %returns the new task selected by the robot
          distanceR(iRobot)=distanceR(iRobot) + norm([posObj(nextTaskAux,:)-posObj(currTaskRobot(iRobot),:)]);
          newCurrTask(iRobot)=nextTaskAux;
      end
     
     currTaskRobot=newCurrTask;
     for i=1:Nrobots
      timesVisitedTask(currTaskRobot(i)) = timesVisitedTask(currTaskRobot(i)) + 1;
     end
      
      if ((min(timesVisitedTask)>0) && allTasksEndT==(-1))
          allTasksEndT=nS;
      end
  end
end