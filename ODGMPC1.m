clc;clear;close all
addpath(genpath('~/Desktop/YALMIP-master'))
%C:\gurobi902\win64\matlab
%addpath(genpath('C:\gurobi902\win64\matlab'))
addpath('Situation');  %% 

%% Load Situation
PP_Simulation = Situation23(); 
%PP_Simulation = Situation11(); %% 차 1대 
% PP_Simulation = Situation12(); %% 
% PP_Simulation = Situation21(); %%
% PP_Simulation = Situation22(); %% 
% PP_Simulation = Situation23(); %% 

%% Goal Info
Ref_V = PP_Simulation.Eco_Car.RefV;

SigmaC = 1.667; %1.6667; 커질수록 차량이 머무는 시간 길어짐  추월 안함  1.8~9 까지 작동안함 , 10~20~30 10단위로 작동함
SigmaL =  0.6385; %%2 주니까 차량 중앙으로 가더니 저속운행 끝

w1 = 0.25;
Risk_Trans = 100*w1*sqrt(pi); %% 리스크 변화량 


%% MPC design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
%%cell 형배열이란 = 인덱싱된 데이터 컨테이너를 사용하는 데이터형 (각 셀에는 모든 데이터형이 포함될수 있음 )
u_MPC  = sdpvar(repmat(nu,1,N+1),repmat(1,1,N+1));  %% sdpvar(repmat 요소값이 nu=2 인 1xn+1 행렬 생성  , 요소값이 1인 1xn+1 행렬 생성)  = 1x11cell
x_MPC  = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1)); %% sdpvar(repmat 요소값이 nu=4 인 1xn+1 행렬 생성  , 요소값이 1인 1xn+1 행렬 생성)
Goal   = sdpvar(repmat(2,1,N) ,repmat(1,1,N));

% Variable
alpha = 1;
kai = 0.5;
gamma = 1;
mu = 0.5;
qu = 0.5;

constraints = [];
objective = 0;

for k = 1 : N     %%mpc 부분 여기 가 OBJECTIVE COSTFUNTION
    objective = objective + alpha*(x_MPC{k}(2) - Goal{k}(1))^2;     % 차선 유지  mpc part
    objective = objective + kai*(x_MPC{k}(3) - Goal{k}(2))^2;       % 속도 유지
    objective = objective + gamma*x_MPC{k}(4)^2;                    % 좌우 움직임 최소
    objective = objective + mu*u_MPC{k}(1)^2;                       % 제어 최소
    objective = objective + qu*u_MPC{k}(2)^2 ;                      % 제어 최소
    
    % 장애물 차량의 운동 방정식을 이용한 향후 경로 예측
    constraints = [constraints, x_MPC{k+1} == A*x_MPC{k} + B*u_MPC{k}]; 
    constraints = [constraints, -0.17*x_MPC{k}(3) <= x_MPC{k}(4) <= 0.17*x_MPC{k}(3)];  %(9)
    
    % 충돌 관련된 제한조건
    constraints = [constraints, PP_Simulation.RLine(2,1) <= x_MPC{k}(2) <= PP_Simulation.LLine(2,1)];  % 차량의 차선 위치 제어

    % 움직임 관련 제한조건
    % 입력
    constraints = [constraints, -4   <= u_MPC{k}(1) <= 2]; % 차량의 입력 값 제어    
    constraints = [constraints, -2   <= u_MPC{k}(2) <= 2]; % 차량의 입력 값 제어
    
    constraints = [constraints, -3   <= u_MPC{k+1}(1)-u_MPC{k}(1) <= 1.5]; % 차량의 입력 값 변위 제어  
    constraints = [constraints, -0.5 <= u_MPC{k+1}(2)-u_MPC{k}(2) <= 0.5]; % 차량의 입력 값 변위 제어
    
    constraints = [constraints, -5    <= x_MPC{k}(2) <=  5];  % 차량의 차선 위치 제어    
    constraints = [constraints,  0    <= x_MPC{k}(3) <= 25];  % 차량의 최대 속도 제어
    constraints = [constraints, -5    <= x_MPC{k}(4) <=  5];  % 차량의 최대 속도 제어
end
parameters_in = {x_MPC{1},[Goal{:}]};
solutions_out = {[u_MPC{:}], [x_MPC{:}]};

%options = sdpsettings('verbose',1,'solver','gurobi','quadprog.maxiter',100); %유료
% this is
options = sdpsettings('verbose',1,'solver','quadprog','quadprog.maxiter',100);
controller_ODG = optimizer(constraints, objective,options,{x_MPC{1},[Goal{:}]},{[u_MPC{:}], [x_MPC{:}]});
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Eco_Car_state = [PP_Simulation.Eco_Car.x; PP_Simulation.Eco_Car.y; PP_Simulation.Eco_Car.vx; PP_Simulation.Eco_Car.vy];
Eco_Car_result = zeros(length(Eco_Car_state),PP_Simulation.SimulationStep);

Obst_Car_state = [PP_Simulation.Obstacle.x; PP_Simulation.Obstacle.y; PP_Simulation.Obstacle.vx; PP_Simulation.Obstacle.vy];

carset = PP_Simulation.Obstacle;

Num_Obstacle = size(PP_Simulation.Obstacle,2);
Obstacle_State = zeros(size(PP_Simulation.Obstacle(1).pos(:,1),1),Num_Obstacle);

%% ODG Value
P_Eco_Car_state = zeros(nx,N);
P_Obs_Car_state = zeros(nx*PP_Simulation.Num_Obstacle,N);

Sensing_step = 0.5;
Sensing_width = [PP_Simulation.RLine(2,1):Sensing_step:PP_Simulation.LLine(2,1)]';

Sensor_Array = zeros(size(Sensing_width,1),PP_Simulation.Num_Obstacle);

Vel_Array = zeros(100,N);

%show
ODG_Value = zeros(length(Sensing_width),PP_Simulation.SimulationStep);
ODG_Value_temp = zeros(length(Sensing_width),PP_Simulation.SimulationStep,PP_Simulation.Num_Obstacle);

Lk = abs(PP_Simulation.RLine(2,1) - PP_Simulation.LLine(2,1));
WL = 100;
w2 = sqrt(Lk*Lk / (32*SigmaL*SigmaL*log(w1) + Lk*Lk));
WM = WL*w1;

WC = 100;
MinDD = 2;%% 필요이유를 모르것음

Index_Iine = 1;
when= zeros(9999,9999);

Eco_Car_result(:,1) = Eco_Car_state;



for step = 1 : PP_Simulation.SimulationStep-N-1
    %% Get Info
    for k = 1 : PP_Simulation.Num_Obstacle
        Obstacle_State(:,k) = PP_Simulation.Obstacle(k).pos(:,step);
    end
    
    %% ODG Method
    % Predect Pos
    k=0;
    if step == 1 || Flag_NaN == 1
        P_Eco_Car_state(:,1) = Eco_Car_state;
        for k = 2 : N+1
            P_Eco_Car_state(:,k) = Dynamic_Model_1(P_Eco_Car_state(:,k-1),[0,0],dT);
        end
        Flag_NaN = 0;
    else
        P_Eco_Car_state = solutions{2};
    end    
    for k = 1 : PP_Simulation.Num_Obstacle
        P_Obs_Car_state(1+4*(k-1):4*k,:) = PP_Simulation.Obstacle(k).pos(:,step:step+N-1);
    end
    
    % ODG Value
    Sensor_Data_N = zeros(size(Sensing_width,1),N);
    Goal_Input = zeros(2,N);
    Goal_Param = zeros(2,1);
    Line_ODG = zeros(2,N,2);
    for MPC_Horison_Step = 1 : N
        V_Pos = P_Eco_Car_state(1:2,MPC_Horison_Step);  %% 내차의 값 (x,y 좌표)
        V_Vel = P_Eco_Car_state(3,MPC_Horison_Step);  %% 내차 속도  (x로 이동방향속도)
        
        Sensor_Data = zeros(size(Sensing_width,1),2);
        Sensor_Data(:,1) = Sensing_width;
        Sensor_TempL = WL * gaussmf(Sensor_Data(:,1),[SigmaL, PP_Simulation.LLine(2,step)]); %% 범위 [시그마L,5]
       % disp([SigmaL, PP_Simulation.LLine(2,step)])
        Sensor_Data(:,2) = Sensor_Data(:,2) + Sensor_TempL;
        Sensor_TempM = WM * gaussmf(Sensor_Data(:,1),[SigmaL*w2, (PP_Simulation.LLine(2,step)+PP_Simulation.RLine(2,step))/2]);
        Sensor_Data(:,2) = Sensor_Data(:,2) + Sensor_TempM;
        Sensor_TempR = WL * gaussmf(Sensor_Data(:,1),[SigmaL, PP_Simulation.RLine(2,step)]);
        Sensor_Data(:,2) = Sensor_Data(:,2) + Sensor_TempR;
        
        Sensor_Array = zeros(size(Sensing_width,1),PP_Simulation.Num_Obstacle);
        % disp(Sensor_TempL);
       % disp(PP_Simulation.LLine(:,2));  
      %disp( PP_Simulation.RLine(2,step));
        %%disp(MPC_Horison_Step);  %%1~10반복
        for k = 1 : PP_Simulation.Num_Obstacle
            O_Pos = P_Obs_Car_state(1+4*(k-1):4*k,MPC_Horison_Step);  %% (1부터 5행까지묶음을) 1~10열의 값을 순차적으로 불러옴  1  즉 장애물 차의 정보 (x,y 방향x속도 y속도)      
            V2O_L = max(abs(O_Pos(1) - V_Pos(1)),MinDD); %%  %%(최대값(절대값(장애물x -내차 x )))
            V2O_V = O_Pos(3) - V_Vel;  
       
            if V2O_L < 50  %%
                    %disp( O_Pos());
                if O_Pos(1) < V_Pos(1) %  
                  
                
                    if V2O_V > 0 % 
                      
                        Ax = WC * V_Vel / V2O_L;
                        Sensor_Array(:,k) = Ax*gaussmf(Sensor_Data(:,1),[SigmaC,O_Pos(2)]);
                        needdata= Sensor_Array(:,k);
                        Sensor_Data(:,2)  = Sensor_Data(:,2) + Sensor_Array(:,k);
                      
                        k=1;
                    end
                     
                else % 
                    if V2O_V < 0 % 
                        Ax = WC * V_Vel / V2O_L;
                        Sensor_Array(:,k) = Ax*gaussmf(Sensor_Data(:,1),[SigmaC,O_Pos(2)]);
                        Sensor_Data(:,2)  = Sensor_Data(:,2) + Sensor_Array(:,k);
                       
                    end
                end
                
            end
            
        end
        Sensor_Data_N(:,MPC_Horison_Step) = Sensor_Data(:,2);
%         Sensor_Data_N(:,MPC_Horison_Step) = Sensor_TempL+Sensor_TempM+Sensor_TempR;
%         Sensor_Data_N(:,MPC_Horison_Step) = Sensor_Array(:,k);
        
        if MPC_Horison_Step == 1
            ODG_Value(:,step) = Sensor_Data(:,2);
            ODG_Value_temp(:,step,:) = Sensor_Array;
        end
        
        % Reference Point Setting
        % Line 1
        [v_MPC_1, l_MPC_1] = min(Sensor_Data_N(1:size(Sensing_width,1)/2-1.5,MPC_Horison_Step));
        Line_ODG(1,MPC_Horison_Step,1) = v_MPC_1;
        Line_ODG(1,MPC_Horison_Step,2) = l_MPC_1+1-1;
        
     
        % Line 2
        [v_MPC_2, l_MPC_2] = min(Sensor_Data_N(size(Sensing_width,1)/2+1.5:end,MPC_Horison_Step));
        Line_ODG(2,MPC_Horison_Step,1) = v_MPC_2;
        Line_ODG(2,MPC_Horison_Step,2) = l_MPC_2+14-1;
       % disp(Line_ODG);
    end
    
    %% Goal decision    
    if sum(Line_ODG(1,:,1))< sum(Line_ODG(2,:,1)) + Risk_Trans
        Index_Iine = 1;
    else
        Index_Iine = 2;
    end
    
    Goal_Input(1,:) = Sensor_Data(Line_ODG(Index_Iine,:,2));
    Goal_Input(2,:) = Ref_V* (100 - Line_ODG(Index_Iine,:,1))./100;
    
    
    %% MPC Control
    inputs = {Eco_Car_state,Goal_Input};
    [solutions, diagnostics] = controller_ODG{inputs};
    Eco_Car_state = Dynamic_Model_1(Eco_Car_state,solutions{1}(:,1)',PP_Simulation.TimeStep);
    
    %% Save    
    Eco_Car_result(:,step) = Eco_Car_state;
    
end
Eco_Car_result = Eco_Car_result(:,1:step);
PP_Simulation.SimulationStep = step;
%% Show
ShowResult
data2=Eco_Car_result;
%plot(Sensor_Data(:,1),Sensor_Data(:,2))

plot(Eco_Car_result(1,:),Eco_Car_result(3,:))
xlim([0 400])

















