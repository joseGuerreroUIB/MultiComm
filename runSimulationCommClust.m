% Execution of a single simulation with an specific parameters. The environment especification should be included in 
% the directory environments.
%Specific for environmenst where the tasks are arranged into clusters. Similar to runMultSimulationComm function.
%Parameters: 
%    - Nt: number of task 
%    - Nrobots: number of robots.
%    - NstepsSim: number of steps of the simulation
%    - nEx: number of the environment to test
%    - maxDistComm: MAX_RANGE
%    - wNrobot: omega_w
% Return: 
%  - distanceR: total distance traveled by each robot (vector)
%  - timesVisitedTask: number of times that a task has been visited (vector)
%  - allTasksEndT: instant when all tasks has been visited.

function [distanceR, timesVisitedTask, allTasksEndT]=runSimulationCommClust(Nt, Nrobots, NstepsSim, nEx, maxDistComm, wNrobot)

  
  %fname=sprintf('exampleObjClust_C%d_C',nEx);
  fname=sprintf('exampleObjClust_C%d_U',nEx);
  load(fname, 'posObjE'); 
  posObj=posObjE(1:Nt,:);
  
  currTaskRobot=repmat(1,1,Nrobots);
  
  newCurrTask=zeros(1,Nrobots);
  timesVisitedTask=zeros(1,Nt); 
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
      for iRobot=1:Nrobots 
          nextTaskAux = nextTaskRobotComm(PF, currTaskRobot(iRobot)); %returns the new task selected by the robot
          %timesVisitedTask(nextTaskAux)=timesVisitedTask(nextTaskAux) + 1;
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