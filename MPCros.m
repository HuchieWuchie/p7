% ROS interface with Matlab
% initilize script with some parameters: 
%exampleHelperROSShutDownSampleNetwork
rosshutdown
clear
%close all 
%Init
Config
MPCdefinition;
%%%% 
%Reference
%%%%

ref = [x_offset+radius*cos(omega*(0:Ts:(T+Hp*Ts))+Phi);...
       y_offset+radius*sin(omega*(0:Ts:(T+Hp*Ts))+Phi)]; % Position
difx=[0-ref(1,1) ref(1,1:end-1)-ref(1,2:end)];
dify=[0-ref(2,1) ref(2,1:end-1)-ref(2,2:end)];
ang=atan2(dify,difx)+pi/2;
ang=ang - 2*pi*floor( (ang+pi)/(2*pi) );
%for i=1:size(ang,2)
%    if ang(1,i)<-3.1
%        ang(1,i)=0;
%    end
%end


ref = [ref;ang]

dref = [ omega*radius*sin(omega*(0:Ts:(T+Hp))+Phi);...
        -omega*radius*cos(omega*(0:Ts:(T+Hp))+Phi)];
% Connect to the ROS on the robot:
rosinit
% start a publisher and a message template on the reset topic
% reset odometry 
[pub,msg] = rospublisher('reset','std_msgs/Empty');
% send a message to reset odometry 
send(pub,msg);
%Subscribe to odometry:
states= rossubscriber('/gazebo/model_states');
%Creat a publisher for heading with a heading msg:
%headingpub = rospublisher('/heading', 'geometry_msgs/Vector3');
%headingmsg = rosmessage(headingpub);
%Create a publisher for velocity with a velocity msg:
velpub = rospublisher('/quadruped/velocity', 'geometry_msgs/Twist');
velmsg = rosmessage(velpub);
%Start the MPC:  %Make sure your robot is on the zero position. if you turn
%on and don't move it from its position then it will be there.
time=0:Ts:T;
%initialize vectors:
X=zeros(3,length(time)); %States [x;y;th] th is the heading angle
V=zeros(1,length(time)); %Velocity magnitude (Velocity in Body frame)
W=zeros(1,length(time)); %Angular Velocity
vx=zeros(1,length(time)); %MPC output of velocity on x
vy=zeros(1,length(time)); %MPC output of velocity on y
w=zeros(1,length(time)); %MPC output of velocity on y
thd=zeros(1,length(time)); %Desired heading angle from MPC
%[ysound, Fs] =audioread('nn.mp3');
%sound(ysound, Fs, 16);
uprev=[0;0;0];
pause(3)
%headingmsg.Z=0;
%send(headingpub,headingmsg)
%heading correction variables
iterations=0;
signed=1;
tru=0;
for k=1:size(ref,2)-Hp
Data=receive(states,1);
%Current x,y, and th
X(1,k)=Data.Pose(2).Position.X;
X(2,k)=Data.Pose(2).Position.Y;
x=Data.Pose(2).Position.X;
y=Data.Pose(2).Position.Y;
%To read the heading: 
qx=Data.Pose(2).Orientation.X;
qy=Data.Pose(2).Orientation.Y;
qz=Data.Pose(2).Orientation.Z;
qw=Data.Pose(2).Orientation.W;
Euler=quat2eul([qx,qy,qz,qw]);
Euler(3);
X(3,k) = Euler(3) - 2*pi*floor( (Euler(3)+pi)/(2*pi) );
Euler(3) - 2*pi*floor( (Euler(3)+pi)/(2*pi) )
%Apply the MPC:
ref(3,k:k+(Hp-1))
%compensation of angle being greater than this
margin=3;
reference_angle=ref(3,k);
iterations=iterations+1;
signed=1;
if margin<abs(reference_angle) & (sign(X(3,k)) ~= sign(reference_angle))
    signed=-1;
end

tic
res=MPCobj({X(1:3,k),uprev,ref(:,k:k+(Hp-1))});
Comptime=toc;
if(Comptime>0.5)
    disp('large time')
    disp(Comptime)
end
u_L = res{1};
u = reshape(u_L, [3, Hu]);
% u_pre(:,:,k) = u;
% P_L = res{2};
% P_pre(:,:,k) = reshape(P_L, [2, Hp]);
uprev=u(:,1);
%u=MPCcode(ref(:,k:k+(H-1)),X(1:2,k),Ts,H); %Pass a reference that matches the sample time of
%MPC.
vx(k)=u(1,1);
v_x=u(1,1);
vy(k)=u(2,1);
v_y=u(2,1);
w(k)=u(3,1);
%velocity interpretation
VxRobot=vx(k)*cos(-X(3,k))-vy(k)*sin(-X(3,k));%/cos(X(3,k));
VyRobot=vx(k)*sin(-X(3,k))+vy(k)*cos(-X(3,k));%/cos(90-X(3,k));
%if abs(v_x)>abs(v_y)
%     V(k)=sign(vx(k))*sqrt(vx(k)^2+sign(vy(k))*vy(k)^2);
%else
%     V(k)=sign(vy(k))*sqrt(sign(vy(k))*vx(k)^2+vy(k)^2);
%end
%V(k)=sqrt(vx(k)^2+vy(k)^2);%vx(k)+vy(k);%u(2,1)+u(1,1);
%Desired heading angle;: ;
%thd(k)=atan2(vy(k),vx(k))-0.7854;
%headingmsg=thd(k);
%velmsg.Linear.X=V(k);
velmsg.Linear.Y=VyRobot;
velmsg.Linear.X=VxRobot*0.97;
cur_heading=w(k);
velmsg.Linear.Z=w(k)*5.77*signed;
%send(headingpub,headingmsg)
send(velpub,velmsg);
pause(Ts-Comptime)
end
%To insure the robot is stopping
velmsg.Linear.Y=0;
velmsg.Linear.X=0;
velmsg.Linear.Z=0;

send(velpub,velmsg);
%exampleHelperROSShutDownSampleNetwork
rosshutdown
%%
figure;
h1=plot(X(1,1:end),X(2,1:end));
hold on 
%h4=plot(X(1,1:Tsim/Ts:end),X(2,1:Tsim/Ts:end),'bX');
%hold on 
h2=plot(ref(1,1:end-Hp),ref(2,1:end-Hp),'g');
hold on 
xlabel('x [m]','interpreter','latex','FontSize',16)
leg=legend('Actualt Trajectory',...
'Desired Trajectory','FontSize',12);
ylabel('y [m]','interpreter','latex','FontSize',16)
title('MPC implementation','interpreter','latex','FontSize',16)
grid on 
leg=legend('Actualt Trajectory',...
'Desired Trajectory');
%set(leg,'interpreter','latex');
%%
h1=plot(X(3,1:end),'g');
hold on 
%h4=plot(X(1,1:Tsim/Ts:end),X(2,1:Tsim/Ts:end),'bX');
%hold on 
h2=plot(ref(3,2:end-Hp));
hold on 
xlabel('Time [s]','interpreter','latex','FontSize',16)
ylabel('$\theta$ [rad]','interpreter','latex','FontSize',16)
%set(leg,'interpreter','latex');