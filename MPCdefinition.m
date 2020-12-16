%clc;clear;close all;
%Define the MPC object with Yalmip:

%Sample time for the MPC controller (Please use this name for sample time): 
Fs = 1;
Ts = 3.3;

%Prediction Horizon (Please use this name prediction horizon)
Hp = 10;

%Control horizon
Hu = 9;

n=3;

refht=sdpvar(n,Hp);     % P_{ref}: The window containing the pos-reference
P0=sdpvar(n,1);         % P(k):    The current state
Uprev=sdpvar(n,1);      % U(k-1):  The previous input command.
deltaU=sdpvar(n*Hu,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLACE YOUR CODE HERE (START)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
A=[1 0 0;0 1 0;0 0 1];%A=[1 0;0 1]

n=size(A,1);

B=[Ts 0 0;0 Ts 0;0 0 Ts];%B=[Ts 0;0 Ts];
m=size(B,2);

%defining Q as the constant punishing the state/reference
Q=[1;1;0.5];%0.5
Q=eye(n).*Q;
Qcell=repmat({Q},1,Hp);
Q_L=blkdiag(Qcell{:});


%Defining the constants for punishing the input
R=[1;1;5];
R=eye(m).*R;
Rcell=repmat({R},1,Hu);
R_L=blkdiag(Rcell{:});

%defining the constant for punishing the slew in the input. Meaning the
%change in input.
Rd=[1;1;20];
Rd=eye(m).*Rd;
Rd_cell=repmat({Rd},1,Hu);
Rd_L=blkdiag(Rd_cell{:});

%%%obtaining the prediction matrices of A, U, Ud. The large caligraphic
%%%matrices)
A_L=gen_A_L(A,Hp);
Bu_L=gen_Bu_L(A,B,Hp);
%couldn't make this function work
%Bud_L=gen_Bud_L(B,Hu,Hp);
Bud_L=find_BDU(Hp,Bu_L,m);
size(Bud_L);
dU_L=gen_dU_L(B,Hu);
size(dU_L);
%The input vector 
input=1;
U=dU_L*[Uprev;deltaU];
[Uprev;deltaU];
%%%% defining the "Large" prediction state space.
P=A_L*P0 + Bu_L*Uprev + Bud_L*deltaU;

ref_h=reshape(refht,[n*size(refht,2) 1]);%reshaping the reference

Cost=(ref_h-P)'*Q_L*(ref_h-P)+U'*R_L*U+deltaU'*Rd_L*deltaU;


%%% Constraints on the controlled variable
% y>-0.6 x=0
%G=constraints_G(Hp);
v_constraints=0.01;
Constraints=[];
Constraints=[Constraints,-v_constraints<=U<=v_constraints];
%Constraints=[Constraints,G*[P;1]<=0];ev,ref(:,k:k+(Hp-1))});




%options = sdpsettings('solver','mosek','verbose',2);
% The Yalmip optimizer-object used for simulation and TurtleBot3 control
MPCobj=optimizer(Constraints,Cost,[],{P0,Uprev,refht},{U, P});

% U: u*: The optimal control window
% P: P*: Optimal states in prediction window

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% YOU CAN PLACE YOUR OWN FUNCTIONS HERE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function cost=cost_prediction(ref,Q,Hp)
sum=zeros(size(2,1),1);
for i=1:Hp
   sum=sum+ref(i)'*Q*ref(i);
end 
cost=sum 
end

function G=constraints_G(Hp)
G=zeros(Hp*2,Hp*2+1);
for i=1:Hp
    G(i*2,i*2)=-1;
    G(i*2,Hp*2+1)=-0.6;
end
end

function F=constraints_u(Hu,lims)
%we both have upper and lower limits for the constraints
F=zeros(Hu*2*2,Hu*2+1);
for i=1:Hu
    %defining limits for u1
    F(i*4-3,i*2-1)=-1;
    F(i*4-2,i*2-1)=1;
    F(i*4-3,Hu*2+1)=-lims;
    F(i*4-2,Hu*2+1)=-lims;
    %defining limits for u2
    F(i*4-1,i*2)=-1;
    F(i*4,i*2)=1;
    F(i*4-1,Hu*2+1)=-lims;
    F(i*4,Hu*2+1)=-lims;
    
end
end

function E=constraints_du(Hu,lims)
%we both have upper and lower limits for the constraints
E=zeros(Hu*2*2,Hu*2+1);
for i=1:Hu
    %defining limits for u1
    E(i*4-3,i*2-1)=-1;
    E(i*4-2,i*2-1)=1;
    E(i*4-3,Hu*2+1)=-lims;
    E(i*4-2,Hu*2+1)=-lims;
    %defining limits for u2
    E(i*4-1,i*2)=-1;
    E(i*4,i*2)=1;
    E(i*4-1,Hu*2+1)=-lims;
    E(i*4,Hu*2+1)=-lims;
    
end
end
function bu=find_BU(Hp,A,B)
    for i=1:(Hp)
        Bu_i = [0 0;0 0];
        for j=1:i
            Bu_i = Bu_i + A*B;
        end
        if i==1
            Bu_=Bu_i;
        else
            Bu_=[Bu_;Bu_i];
        end
    end
    bu=Bu_;
end

function bdu=find_BDU(Hp,Bu_,n)
    k=size(Bu_,1);
    for i=1:(Hp-1)
        if i==1
            Bdu_=Bu_;
        else
            z=zeros((i-1)*2,n);
            B_temp=Bu_(1:(k-i*2+2),:);
            Bdu_=[Bdu_ [z;B_temp]];
        end 
    end
    bdu=Bdu_;
end


%% functions from slide...
function [A_L]=gen_A_L(A,hp)
    n=size(A,1);
    A_L=zeros(n*hp,n);
    Ai=A;
    for i=1:hp
        if 1==i
            Ai=A;
        else
            Ai=Ai*A;
        end
        A_L((i-1)*n+1:i*n,:)=Ai;
    end
end

function [Bu_L]=gen_Bu_L(A,B,hp)
    n=size(A,1);
    m=size(B,2);
    Bu_L = zeros(n*hp,m);
    Bi = zeros(n,m);
    for i = 1:hp
        Bi = A*Bi + B;
        Bu_L((i-1)*n+1:i*n,:) = Bi;
    end
end

function [Bud_L] = gen_Bud_L(Bu_L,hu,hp)
    n = size(Bu_L,1)/hp;
    m = size(Bu_L,2);
    Bud_L = zeros(n*hp,m*hu);
    for i = 1:hu
        Bud_L((i-1)*n+1:end,(i-1)*m+1:i*m) = Bu_L(1:end-n*(i-1),:);
    end
end

function [G_L] = gen_G_L(ylim,hp)
    F=[0 -1]
    Fcell = repmat((F),[1 hp]);
    F_L = blkdiag(Fcell(:));
    G_L = [F_L ones(hp,1)*ylim]
end

function [dU_L] = gen_dU_L(B,hu)
    m = size(B,2);
    Iu = eye(m,m);
    F = ones(hu,hu+1);
    F_tri = tril(F,1);
    dU_L = kron(F_tri, Iu);
end