%Run multiple simulations with communication between robots
%The final results are saved in the resCommDirectory with the name res_
%Parameters: 
%   - NMultSim number of simulations for each combination of parameters
%   - Nt: number of tasks
%   - Nrobots: number of robots
%   - NstepsSim: Maximum number of steps for each simuation
%   - nEx: number of envoriments for each simulation and set of parameters.
%   - maxDistComm: vector with the maximum range distances to test
%   - wNrobot: vector with the values of the parameter omega_w
%   - sub: sufix of the file name with the final results
% Return:
%  - MdistR: vector with the total distance traveled for each robot.
%  - MallTasksEndT: vector with the number of steps required to visit all the tasks
function [MdistR, MallTasksEndT]=runMultSimulationComm(NMultSim,Nt, Nrobots, NstepsSim, nEx, maxDistComm, wNrobot,sub)
  MdistR = [];
  MallTasksEndT = [];
  for maxDistComm_i=maxDistComm
  for wNrobot_i=wNrobot
  for j=nEx
     for i=1:NMultSim
         [distanceR, timesVisitedTask, allTasksEndT]=runSimulationComm(Nt, Nrobots, NstepsSim, j, maxDistComm_i, wNrobot_i);
         MdistR = [MdistR; maxDistComm_i, wNrobot_i, j, i, sum(distanceR)];
         MallTasksEndT = [MallTasksEndT; maxDistComm_i, wNrobot_i, j, i, allTasksEndT];
     end
  end
  end
  end
  fname=sprintf('resComm/res_%d_%d_%d_%d',Nt,Nrobots,sub);
  save(fname);
end