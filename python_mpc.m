%% Goal Info
Ref_V = 50;
w1 = 0.25;


%% MPC design
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
dT = 0.05;
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
