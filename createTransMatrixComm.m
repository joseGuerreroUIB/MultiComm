%Create a possibilistic (PijF1) and probabilsitic function (PijProb1)
%PosObj: position of the objects
%errorProb: error added to the transitions.
%TypeFunc: 2 for logistic, 1 for standar
%nV: power of the function. P.e: e^(-d^n*TH^n)
%TH1: threshold value
%taskRobot: task assigned to each robot 
function [PijF1, PijProb1]=createTransMatrixComm(posObj, Nt, typeFun, nV, TH, taskRobot, maxDistComm, paramNR)

    distObj=zeros(Nt,Nt);
    nRobotsObj=zeros(1,Nt);
    
    Nr = length(taskRobot);
    taskRobot=taskRobot(taskRobot > 0); %Nomes robots amb taska assignada
    NrT = length(taskRobot);
    
    posTaskRobot=zeros(Nr,2);
    
    
    %Calculate the number of robots 
%     Nr = length(taskRobot);
%    for j=1:Nr
%       posRj = posObj(taskRobot(j));
%       posRjRep = repmat(posRj,Nt,1);
%       indexT=find(norm(posRjRep,opsObj)>=maxDist);
%       nRobotsObj(indexT) = nRobotsObj(indexT) + 1;
%    end
    
    posTaskRobot=posObj(taskRobot,:);
    disAux=zeros(1,NrT);
    for i=1:Nt
        posObji=repmat(posObj(i,:),NrT,1);
        for j=1:NrT
          disAux(j)=norm(posTaskRobot(j,:)-posObji(j,:));
        end
        %indexT=find(pdist2(posObji,posTaskRobot)>=maxDistComm);
        nRobotsObj(i) = nRobotsObj(i) + sum(disAux <= maxDistComm);
    end

    %nRobotsObj=nRobotsObj./NrT;
    
    %errorR=unifrnd(-errorProb, errorProb,1,Nt);
    for i=1:Nt
       %Get the distance from object ith object to any other one
       currObj=repmat(posObj(i,:),Nt,1);
       distAux = (posObj-currObj).^2;
       distAux = sum(distAux');
       distAux = sqrt(distAux);
       distObj(i,:) = distAux;
       distObj(i,i) = 0; %Remove 
       CUobj=repmat(nRobotsObj(i),1,Nt);
       %if (typeU==1)
       distObj(i,:) = paramNR*max(nRobotsObj-CUobj,0) + distObj(i,:);
       %else
       %   distObj(i,:) = max((CUobj-Uobj) + distObj(i,:),0);
      % end
    end

    %Create Probability Transition matrix

    PijF1=distObj;
    PijProb1=distObj;
    for i=1:Nt
       switch typeFun
           case 2 %logistic function 
               PijF1(i,:) = exp(-(TH*PijF1(i,:)).^nV);
           otherwise
                PijF1(i,:)=1./(1+(PijF1(i,:).^nV)*TH.^nV);
       end
       

       PijProb1(i,:)=PijF1(i,:)./sum(PijF1(i,:));
    end
end