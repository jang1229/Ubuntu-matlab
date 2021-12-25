%clc;clear;close all

%C:\gurobi902\win64\matlab
%addpath(genpath('C:\gurobi902\win64\matlab'))
addpath('Situation');  %% 
addpath('ros');



%% Goal Info
Ref_V = 50;
w1 = 0.25;
Risk_Trans = 100*w1*sqrt(pi);


%% 

PP_Simulation.Eco_Car.Size(2) =10;
PP_Simulation.Eco_Car.Size(1) =10;

%%
PP_Simulation.Eco_Car.x = 0;
PP_Simulation.Eco_Car.y=0;
PP_Simulation.Eco_Car.vx = 50;
PP_Simulation.Eco_Car.vy = 0;
PP_Simulation.TimeStep = 0.1;
PP_Simulation.SimulationStep=1500;
%%%
%% MPC design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%odg_xy_Sub.LatestMessage.Data;
%double(odg_xy_Sub.LatestMessage.Data(1,1));
dT = PP_Simulation.TimeStep; 
N = 10;

A = [1, 0, dT, 0;
     0, 1, 0, dT;
     0, 0, 1, 0;
     0, 0, 0, 1];
B = [0,0;
     0,0;
     1,0;
     0,1];
% Initial state
nx = size(A,2);  %%4 
nu = size(B,2);  %%= 2
%%cell
u_MPC  = sdpvar(repmat(nu,1,N+1),repmat(1,1,N+1));  %% 
x_MPC  = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1)); %% 
Goal   = sdpvar(repmat(2,1,N) ,repmat(1,1,N));

% Variable
alpha = 1;
kai = 0.5;
gamma = 1;
mu = 0.5;
qu = 0.5;

constraints = [];
objective = 0;

for k = 1 : N     %%mpc 
    objective = objective + alpha*(x_MPC{k}(2) - Goal{k}(1))^2;     % 차선 유지  mpc part
    objective = objective + kai*(x_MPC{k}(3) - Goal{k}(2))^2;       % 속도 유지
    objective = objective + gamma*x_MPC{k}(4)^2;                    % 좌우 움직임 최소
    objective = objective + mu*u_MPC{k}(1)^2;                       % 제어 최소
    objective = objective + qu*u_MPC{k}(2)^2 ;                      % 제어 최소
    
    % 장애물 차량의 운동 방정식을 이용한 향후 경로 예측
    constraints = [constraints, x_MPC{k+1} == A*x_MPC{k} + B*u_MPC{k}]; 
    constraints = [constraints, -0.17*x_MPC{k}(3) <= x_MPC{k}(4) <= 0.17*x_MPC{k}(3)];  %(9)
    
    % 충돌 관련된 제한조건
    %constraints = [constraints, double(odg_xy_Sub.LatestMessage.Data(1,1)) <= x_MPC{k}(2) <= double(odg_xy_Sub.LatestMessage.Data(2,1))];  % 차량의 차선 위치 제어
    constraints = [constraints,  -100<= x_MPC{k}(2) <= 100];  % 차량의 차선 위치 제어

    % 움직임 관련 제한조건
    % 입력
    constraints = [constraints, -40   <= u_MPC{k}(1) <= 20]; % 차량의 입력 값 제어    
    constraints = [constraints, -20   <= u_MPC{k}(2) <= 20]; % 차량의 입력 값 제어
    
    constraints = [constraints, -30   <= u_MPC{k+1}(1)-u_MPC{k}(1) <= 15]; % 차량의 입력 값 변위 제어  
    constraints = [constraints, -5 <= u_MPC{k+1}(2)-u_MPC{k}(2) <= 5]; % 차량의 입력 값 변위 제어
    
    constraints = [constraints, -100    <= x_MPC{k}(2) <=  100];  % 차량의 차선 위치 제어    
    constraints = [constraints,  30    <= x_MPC{k}(3) <= 60];  % 차량의 최대 속도 제어
    constraints = [constraints, -100    <= x_MPC{k}(4) <=  100];  % 차량의 최대 속도 제어
end
parameters_in = {x_MPC{1},[Goal{:}]};
solutions_out = {[u_MPC{:}], [x_MPC{:}]};

%options = sdpsettings('verbose',1,'solver','gurobi','quadprog.maxiter',100); %유료
% this is
options = sdpsettings('verbose',1,'solver','quadprog','quadprog.maxiter',100);
controller_ODG = optimizer(constraints, objective,options,{x_MPC{1},[Goal{:}]},{[u_MPC{:}], [x_MPC{:}]});
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%

Eco_Car_state = [PP_Simulation.Eco_Car.x; PP_Simulation.Eco_Car.y; PP_Simulation.Eco_Car.vx; PP_Simulation.Eco_Car.vy];
Eco_Car_result = zeros(length(Eco_Car_state),PP_Simulation.SimulationStep);

%Obst_Car_state = [PP_Simulation.Obstacle.x; PP_Simulation.Obstacle.y; PP_Simulation.Obstacle.vx; PP_Simulation.Obstacle.vy];

%carset = PP_Simulation.Obstacle;

%Num_Obstacle = size(PP_Simulation.Obstacle,2);
%Obstacle_State = zeros(size(PP_Simulation.Obstacle(1).pos(:,1),1),Num_Obstacle);

%% code part 
N=10;
Goal_Input = zeros(2,N);
Goal_Param = zeros(2,1);
Sensor_Data_N = zeros(321,N);
Sensor_Data= zeros(321,2);
Line_ODG = zeros(2,N,2);

P_Eco_Car_state = zeros(nx,N);
%Sensor_Data_N();
ODG_Value = zeros(length(Sensing_width),PP_Simulation.SimulationStep);

Eco_Car_result(:,1) = Eco_Car_state;


V_Pos =  [0;0];%% 내차의 값 (x,y 좌표)
V_Vel = 60;  %% 내차 속도  (x로 이동방향속도)


%% attrcative potential
Sensing_step=1;
Sensing_width = [-160:Sensing_step:160]';

end_point(2)=0;

by=150;
check =zeros(1,3);
senter =95;
%%
for step = 1 : PP_Simulation.SimulationStep
    


    PP_Simulation.LLine(1,step) =step;
    PP_Simulation.RLine(1,step) =step;
 %   k=0;
  %  PP_Simulation.canter(step,1) = (PP_Simulation.LLine(2,:)+PP_Simulation.RLine(2,:))./2,PP_Simulation.LLine(1,:);
  %  end_point(1)= PP_Simulation.canter;
    check(1,1) =  Eco_Car_state(3,1); % x 축 속도 
    check(1,2) =  Eco_Car_state(2,1); % x 좌표 
    check(1,3) =  Eco_Car_state(4,1); % y 축 속도 
    
    
    
  
    %data.Data =check(1,1)
   
    
    %data.Data(1.2) = Eco_Car_state(4,1);
   % send (mpcPub,data);
   

    
    
    
    if step == 1 || Flag_NaN == 1
        P_Eco_Car_state(:,1) = Eco_Car_state;
        for k = 2 : N+1
            P_Eco_Car_state(:,k) = Dynamic_Model_1(P_Eco_Car_state(:,k-1),[0,0],dT);
        end
        Flag_NaN = 0;
    else
        P_Eco_Car_state = solutions{2};
    end    
%    for k = 1 : PP_Simulation.Num_Obstacle
%        P_Obs_Car_state(1+4*(k-1):4*k,:) = PP_Simulation.Obstacle(k).pos(:,step:step+N-1);
%    end

   
   
   for MPC_Horison_Step = 1 : N
        Sensor_Data = zeros(size(Sensing_width,1),2);
        Sensor_Data(:,1) = Sensing_width;
        Sensor_Data(:,2) = odg_s1.LatestMessage.Data;
        V_Pos =  P_Eco_Car_state(1:2,MPC_Horison_Step);%% 내차의 값 (x,y 좌표)
        V_Vel = P_Eco_Car_state(3,MPC_Horison_Step);   %% 내차 속도  (x로 이동방향속도)
         % Sensor_Data = zeros(size(Sensing_width,1),2);
          
      %  odg_data=odg_s1.LatestMessage.Data; %% 내부 Message 정보확인 
      %  Sensor_Data(:,2) =odg_data;
         linek= double(odg_xy_Sub.LatestMessage.Data);
     %   Sensor_Data_N(:,MPC_Horison_Step) = Sensor_Data(:,2);
        
         Sensor_Data_N(:,N) = odg_s1.LatestMessage.Data;
  
      %   Sensor_Data_N(:,1) = odg_s1.LatestMessage.Data;
       %  Sensor_Data_N(:,2) = Sensor_Data_N(:,1);
        % Sensor_Data_N(:,3) = Sensor_Data_N(:,1);
         %Sensor_Data_N(:,4) = Sensor_Data_N(:,1);
         %Sensor_Data_N(:,5) = Sensor_Data_N(:,1);
         %Sensor_Data_N(:,6) = odg_s2.LatestMessage.Data;
         %Sensor_Data_N(:,7) = Sensor_Data_N(:,6);
         %Sensor_Data_N(:,8) = Sensor_Data_N(:,6); 
         %Sensor_Data_N(:,9) = Sensor_Data_N(:,6);
         %Sensor_Data_N(:,10) = Sensor_Data_N(:,6);
         
            LLine(:,1) =linek(2,1);
         RRine(:,1) =linek(1,1);
         LLine(:,2) =linek(2,1);
         RRine(:,2)=linek(1,1);
         
         LLine(:,3) =linek(4,1);
         RRine(:,3) =linek(3,1);
         LLine(:,4) =linek(4,1);
         RRine(:,4) =linek(3,1);
         
         LLine(:,5) =linek(6,1);
         RRine(:,5) =linek(5,1);
         LLine(:,6) =linek(6,1);
         RRine(:,6) =linek(5,1);
         LLine(:,7) =linek(8,1);
         RRine(:,7) =linek(7,1);
         LLine(:,8) =linek(8,1);
         RRine(:,8) =linek(7,1);
         LLine(:,9) =linek(8,1);
         RRine(:,9) =linek(7,1);
         LLine(:,10) =linek(8,1);
         RRine(:,10) =linek(7,1);
        
        if MPC_Horison_Step == 1
            ODG_Value(:,step) =  Sensor_Data_N(:,N);
            PP_Simulation.LLine(2,step)= LLine(:,N);
            PP_Simulation.RLine(2,step)= RRine(:,N);
            %ODG_Value_temp(:,step,:) = Sensor_Array;
        end
        
        
        
        % Reference Point Setting 기준점 설정 
         % Line one
        [v_MPC_1, l_MPC_1] = min(Sensor_Data_N(1:321,MPC_Horison_Step));
        Line_ODG(1,MPC_Horison_Step,1) = v_MPC_1;
        Line_ODG(1,MPC_Horison_Step,2) = l_MPC_1+1-1;
        % Line 1
       % [v_MPC_1, l_MPC_1] = min(Sensor_Data_N(1:321/2-1.5,MPC_Horison_Step));
       % Line_ODG(1,MPC_Horison_Step,1) = v_MPC_1;
       % Line_ODG(1,MPC_Horison_Step,2) = l_MPC_1+1-1;
        
     
        % Line 2
       % [v_MPC_2, l_MPC_2] = min(Sensor_Data_N(321/2+1.5:end,MPC_Horison_Step));
       % Line_ODG(2,MPC_Horison_Step,1) = v_MPC_2;
       % Line_ODG(2,MPC_Horison_Step,2) = l_MPC_2+14-1;
       % disp(Line_ODG);

   end
    senter = (linek(2,1)+linek(1,1))/2;
    senter=round(senter);
    
      % b= round( (sqrt((senter-160).^2+(by.^2))));
       %c=round(sqrt(senter.^2+by.^2));
      % a=160;
       
    R=atan2(by,senter); 
    D = rad2deg(R);
       %angle =(b.^2-a.^2-c.^2)/2*a*b;
       %angle=acosd(angle)
    if D <= 60
        D=60;
    end
       
    if D>=130
        D=130;
    end
       
    check(1,2) =D;
    
    data.Data =check
    %send (anlgePub,data);
    send (mpcPub,data);
    


    %% Goal decision    
    %if sum(Line_ODG(1,:,1))< sum(Line_ODG(2,:,1)) + Risk_Trans
        Index_Iine = 1;
   % else
    %    Index_Iine = 2;
   % endplot(Eco_Car_result(1,:),Eco_Car_result(3,:))  
    
    Goal_Input(1,:) = Sensor_Data(Line_ODG(Index_Iine,:,2));
    Goal_Input(2,:) = Ref_V* (100 - Line_ODG(Index_Iine,:,1))./100;
   % Goal_state(:,step) =  Goal_Input(:,N);
    
    %% MPC Control
    inputs = {Eco_Car_state,Goal_Input};
    [solutions, diagnostics] = controller_ODG{inputs};
    Eco_Car_state = Dynamic_Model_1(Eco_Car_state,solutions{1}(:,1)',PP_Simulation.TimeStep);

    %% Save    
    Eco_Car_result(:,step) = Eco_Car_state;

    %disp(solutions{1,2});
    
end
Eco_Car_result = Eco_Car_result(:,1:step);
PP_Simulation.SimulationStep = step;
%% Show

data2=Eco_Car_result;
%plot(atan2( Eco_Car_result(4,:), Eco_Car_result(3,:)))
plot(Eco_Car_result(1,:),Eco_Car_result(3,:))  
%Showros

test123