%Create a possibilistic (PijF1) and probabilsitic function (PijProb1)
%PosObj: position of the objects
%errorProb: error added to the transitions.
%TypeFunc: 2 for logistic, 1 for standar
%nV: power of the function. P.e: e^(-d^n*TH^n)
%TH1: threshold value
%Uobj: nx1 Objects' utilities
function [PijF1, PijProb1]=createTransMatrix(posObj, Nt, errorProb, typeFun, nV, TH, Uobj, typeU)

    distObj=zeros(Nt,Nt);
    errorR=unifrnd(-errorProb, errorProb,1,Nt);
    for i=1:Nt
       %Get the distance from object ith object to any other one
       currObj=repmat(posObj(i,:),Nt,1);
       distAux = (posObj-currObj).^2;
       distAux = sum(distAux');
       distAux = sqrt(distAux);
       distObj(i,:) = distAux+errorR.*distAux;
       distObj(i,i) = 0; %Remove 
       CUobj=repmat(Uobj(i),1,Nt);
       if (typeU==1)
         distObj(i,:) = max(CUobj-Uobj,0) + distObj(i,:);
       else
          distObj(i,:) = max((CUobj-Uobj) + distObj(i,:),0);
       end
    end

    %Create Probability Transition matrix
    %robotTH=1/(283*sqrt(2)/4);
    %robotTHExp = 0.0098;
    PijF1=distObj;
    PijProb1=distObj;
    for i=1:Nt
       %PijF1(i,i)=min(distObj(i,distObj(i,:)>0));
       switch typeFun
           case 2 %logistic function 
               PijF1(i,:) = exp(-(TH*PijF1(i,:)).^nV);
           otherwise
                PijF1(i,:)=1./(1+(PijF1(i,:).^nV)*TH.^nV);
       end
       
       errorR=unifrnd(-errorProb, errorProb);
       PijF1(i,:) = PijF1(i,:) + errorR * PijF1(i,:);
       PijProb1(i,:)=PijF1(i,:)./sum(PijF1(i,:));
    end
end