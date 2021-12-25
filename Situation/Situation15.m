clc; clear;close all

x = 0:0.2:1000;
y = -10:0.2:10;

Time = 100;
Ref_V = 18;

CarSize = [5; 2.5]; % European Van
deltaT = 0.1;

SigmaC = 1.916*7;
WC = 140;
WL = 90;
MinDD = 2;
Risk_Trans = 10;

%% Load obstacle
Left_Line_Pos = 6;
Obs_Left_Line = [x',Left_Line_Pos*ones(length(x),1)];

Right_Line_Pos = -6;
Obs_Right_Line = [x',Right_Line_Pos*ones(length(x),1)];


%% Load Compaire Situation
File_Name = 'Sample1'

%% Car obstacle
Eco_Car_state   = [  0; -3; 20; 0];
Obstacle1_state = [ 50; -3; 15; 0]; % x,y, vx, vy
Obstacle2_state = [-20; 3; 15; 0];


%% Save Array
Eco_Car_result = zeros(length(Obstacle1_state),Time);
Obstacle1_result = zeros(length(Obstacle1_state),Time);
Obstacle2_result = zeros(length(Obstacle2_state),Time);

Sensor_Angle = -30:0.5:30;
Sensor_Data = zeros(length(Sensor_Angle),1);
ODG_Value = zeros(length(Sensor_Angle),Time);
ODG_Value_temp = zeros(length(Sensor_Angle),Time,3);

%% Obstacle Moving Plan
% Car 1 : Cut-in
U_Obstacle1_Plan = zeros(2,Time+1);
U_Obstacle1_Plan(:,1) = Obstacle1_state(1:2);
for j = 2:Time
    U_Obstacle1_Plan(1,j) = U_Obstacle1_Plan(1,j-1) + Obstacle1_state(3)*deltaT;
    U_Obstacle1_Plan(2,j) = U_Obstacle1_Plan(2,j-1) + Obstacle1_state(4)*deltaT;
end

% Car 2 : Cut-out
U_Obstacle2_Plan = zeros(2,Time+1);
U_Obstacle2_Plan(:,1) = Obstacle2_state(1:2);
for j = 2:Time
    U_Obstacle2_Plan(1,j) = U_Obstacle2_Plan(1,j-1) + Obstacle2_state(3)*deltaT;
    U_Obstacle2_Plan(2,j) = U_Obstacle2_Plan(2,j-1) + Obstacle2_state(4)*deltaT;
end


%% MPC
dT = 0.1;
A = [1, 0, dT, 0;
     0, 1, 0, dT;
     0, 0, 1, 0;
     0, 0, 0, 1];
B = [0,0;
     0,0;
     1,0;
     0,1];
nx = 4;
nu = 2;
N = 20;
% Initial state
u_MPC  = sdpvar(repmat(nu,1,N+1),repmat(1,1,N+1));
x_MPC  = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
Goal = sdpvar(repmat(2,1,N+1),repmat(1,1,N+1));
constraints = [];
objective = 0;

for k = 1 : N
    objective = objective + (x_MPC{k}(2) - Goal{k+1}(2))^2;  % 차선 유지
    objective = objective + (x_MPC{k}(3) - Ref_V)^2;         % 속도 유지
    objective = objective + (u_MPC{k})'*(u_MPC{k});          % 제어 최소
    
    % 장애물 차량의 운동 방정식을 이용한 향후 경로 예측 // 
    constraints = [constraints, x_MPC{k+1} == A*x_MPC{k} + B*u_MPC{k}]; 

    constraints = [constraints, -3   <= u_MPC{k}(1) <= 0.7];  % 차량의 입력 값 제어
%     constraints = [constraints, -0.1 <= u_MPC{k}(2) <= 0.1];  % 차량의 입력 값 제어    
    constraints = [constraints, -0.5 <= u_MPC{k}(2) <= 0.5];  % 차량의 입력 값 제어    
    constraints = [constraints, -6   <= x_MPC{k}(2) <=   6];  % 차량의 차선 위치 제어
    constraints = [constraints,  0   <= x_MPC{k}(3) <=   6];  % 차량의 최대 속도 제어
    constraints = [constraints, -0.5 <= x_MPC{k}(4) <= 0.5];  % 차량의 최대 속도 제어
end

parameters_in = {x_MPC{1},[Goal{:}]};
solutions_out = {[u_MPC{:}], [x_MPC{:}]};

options = sdpsettings('verbose',1,'solver','gurobi','quadprog.maxiter',100);
% options = sdpsettings('verbose',1,'solver','quadprog','quadprog.maxiter',100);
% controller = optimizer(constraints, objective,options,parameters_in,solutions_out);
controller = optimizer(constraints, objective,options,{x_MPC{1},[Goal{:}]},{[u_MPC{:}], [x_MPC{:}]});

%% Eco Car
P_Eco_Car_state = zeros(nx,N+1);
P_Obstacle1_state = zeros(nx,N+1);
P_Obstacle2_state = zeros(nx,N+1);
P_Obstacle3_state = zeros(nx,N+1);

Index_Iine = 1;
Flag_NaN = 0;
tic
for step = 1 : Time - N - 1
%     step
    Eco_Car_result(:,step) = Eco_Car_state;
    Obstacle1_result(:,step) = Obstacle1_state;
    Obstacle2_result(:,step) = Obstacle2_state;
    % Predect Pos
    if step == 1 || Flag_NaN == 1
        P_Eco_Car_state(:,1) = Eco_Car_state;
        for k = 2 : N+1
            P_Eco_Car_state(:,k) = Dynamic_Model_1(P_Eco_Car_state(:,k-1),[0,0],dT);
        end
        Flag_NaN = 0;
    else
        P_Eco_Car_state = solutions{2};
    end
    
    P_Obstacle1_state = U_Obstacle1_Plan(:,step:step+N);
    P_Obstacle2_state = U_Obstacle2_Plan(:,step:step+N);
    
    % ODG Value
    Sensor_Data_N = zeros(size(Sensor_Data,1),N-1);
    for k = 1 : N+1
        V_Pos = P_Eco_Car_state(1:2,k);
        V_Vel = P_Eco_Car_state(3,k);
        
        Sigma = 15;
        Lk = 30;
        mmm = sqrt(Lk*Lk / (2*log(8)*Sigma*Sigma + Lk*Lk));
        Sensor_Data = zeros(121,2);
        Sensor_Data(:,1) = -60:60;
        Sensor_TempL = WL * gaussmf(Sensor_Data(:,1),[Sigma*mmm, 60]);
        Sensor_Data(:,2) = Sensor_Data(:,2) + Sensor_TempL;
        Sensor_TempM = WL*0.25* gaussmf(Sensor_Data(:,1),[Sigma, 0]);
        Sensor_Data(:,2) = Sensor_Data(:,2) + Sensor_TempM;
        Sensor_TempR = WL * gaussmf(Sensor_Data(:,1),[Sigma*mmm,-60]);
        Sensor_Data(:,2) = Sensor_Data(:,2) + Sensor_TempR;
        
%         O_Pos = Obstacle1_state(1:2);
        O_Pos = P_Obstacle1_state(:,k);
        V2O_L = max(abs(O_Pos(1) - V_Pos(1)),MinDD);%norm(O_Pos - V_Pos);
        Sensor_Temp1 = zeros(size(Sensor_Data,1),1);
        if V2O_L < 60 && (O_Pos(1) + CarSize(1)*1 > V_Pos(1))
            Ax = WC * V_Vel / V2O_L;
%             SigmaC = 1.916*3;
            Sensor_Temp1 = Ax*gaussmf(Sensor_Data(:,1),[SigmaC,O_Pos(2)*10]);
            Sensor_Data(:,2) = Sensor_Data(:,2) + Sensor_Temp1;
        end
        O_Pos = P_Obstacle2_state(:,k);
%         O_Pos = Obstacle2_state(1:2);
        V2O_L = max(abs(O_Pos(1) - V_Pos(1)),MinDD);%norm(O_Pos - V_Pos);
        Sensor_Temp2 = zeros(size(Sensor_Data,1),1);
        if V2O_L < 60 && (O_Pos(1) + CarSize(1)*2 > V_Pos(1))
            Ax = WC * V_Vel / V2O_L;
%             SigmaC = 1.916*3;
            Sensor_Temp2 = Ax*gaussmf(Sensor_Data(:,1),[SigmaC,O_Pos(2)*10]);
            Sensor_Data(:,2) = Sensor_Data(:,2) + Sensor_Temp2;
        end
        Sensor_Data_N(:,k) = Sensor_Data(:,2);
    end
    % Reference Point Setting
    Goal_Input = zeros(2,N+1);
    Goal_Param = zeros(2,1);
    for k = 1 : N+1
        [v_MPC, l_MPC] = findpeaks(-Sensor_Data_N(:,k));
        
        Goal_Input(1,k) = P_Eco_Car_state(1,k);
        Goal_Param(1) = Goal_Param(1) - v_MPC(1);
        if length(v_MPC) == 2
            Goal_Param(2) = Goal_Param(2) - v_MPC(2);
        else
            Goal_Param(2) = Goal_Param(2) - v_MPC(1);
        end
        if length(v_MPC) == 2
            Index_Iine_temp = Index_Iine;
%             Index_Iine_temp_before = Index_Iine_before;
        else
            Index_Iine_temp = 1;
        end
        
        Goal_Input(2,k) = Sensor_Data(l_MPC(Index_Iine_temp),1)/10;
        
%         if Flag_LineChange == 1
%             
%         end
        
    end
    Present_Line = round(Eco_Car_state(2)*10) + 61;
%     if V_Pos(2) < 0
%         Risk_Trans = max(Sensor_Data_N(Present_Line:30+61,1));
%     else
%         Risk_Trans = max(Sensor_Data_N(-30+61:Present_Line,1));
%     end
%     Risk_Trans
     
    if Index_Iine == 1
        if Goal_Param(1) > Goal_Param(2) + Risk_Trans
            Index_Iine = 2;
            Index_Iine_before = 1;
            Flag_LineChange = 1;
        end
    elseif Index_Iine == 2
        if Goal_Param(2) > Goal_Param(1) + Risk_Trans
            Index_Iine = 1;
            Index_Iine_before = 2;
            Flag_LineChange = 1;
        end
    end
    
%     if (Flag_LineChange == 1)
%         if Index_Iine == 1
%            for k = 1 : N+1
%               
%            end 
%         else
%             
%         end
%     end
    
    % MPC running
    inputs = {Eco_Car_state,Goal_Input};
    [solutions, diagnostics] = controller{inputs};
    
    Eco_Car_state_pre = Eco_Car_state;
    Eco_Car_state = Dynamic_Model_1(Eco_Car_state,solutions{1}(:,1)',dT);
    
    if isnan(Eco_Car_state(3)) == 1
        Eco_Car_state(3) = Eco_Car_state_pre(3);
        Eco_Car_state(4) = 0;%Eco_Car_state_pre(4);
        Flag_NaN = 1;
        step
    end
    
    Obstacle1_state(1:2) = U_Obstacle1_Plan(:,step);
    Obstacle2_state(1:2) = U_Obstacle2_Plan(:,step);
%     
    ODG_Value(:,step) = Sensor_Data(:,2);
    ODG_Value_temp(:,step,1) = Sensor_Temp1;
    ODG_Value_temp(:,step,2) = Sensor_Temp2;
    
    Eco_Car_path(:,step) = Goal_Input(:,1);
    
end
simtime = toc

%% Show
figure(1)
clf
xlim([x(1), x(end)])
ylim([y(1), y(end)])
hold on
plot(Obs_Left_Line(:,1),Obs_Left_Line(:,2),'k');
plot(Obs_Right_Line(:,1),Obs_Right_Line(:,2),'k');


plot(Eco_Car_result(1,:),Eco_Car_result(2,:),'*k');
plot(Obstacle1_result(1,:),Obstacle1_result(2,:),'*b');
plot(Obstacle2_result(1,:),Obstacle2_result(2,:),'*r');
% plot(Obstacle3_result(1,:),Obstacle3_result(2,:),'*g');

% return
% Show result
figure(2)
figure(4)
for step = 1 : Time - N -1
    Eco_Car_state = Eco_Car_result(:,step);
    Obstacle1_state = Obstacle1_result(:,step);
    Obstacle2_state = Obstacle2_result(:,step);
%     Obstacle3_state = Obstacle3_result(:,step);
    
    
    % Default situation
    figure(2)
    clf
    xlim([Eco_Car_state(1)-20, Eco_Car_state(1)+50])
    ylim([-2.5, 7.5])
    axis equal
    grid on
    hold on
    plot(Obs_Left_Line(:,1),Obs_Left_Line(:,2),'k','Linewidth',1.5);
    plot(Obs_Right_Line(:,1),Obs_Right_Line(:,2),'k','Linewidth',1.5);
    
    rectangle('Position',[Eco_Car_state(1)-0.5*CarSize(1), Eco_Car_state(2)-0.5*CarSize(2), CarSize(1), CarSize(2)]);
    rectangle('Position',[Obstacle1_state(1)-0.5*CarSize(1), Obstacle1_state(2)-0.5*CarSize(2), CarSize(1), CarSize(2)],'FaceColor','g');
    rectangle('Position',[Obstacle2_state(1)-0.5*CarSize(1), Obstacle2_state(2)-0.5*CarSize(2), CarSize(1), CarSize(2)],'FaceColor','g');
%     rectangle('Position',[Obstacle3_state(1)-0.5*CarSize(1), Obstacle3_state(2)-0.5*CarSize(2), CarSize(1), CarSize(2)],'FaceColor','g');
    
    % Collision Detection
    Flag_Collision = 0;
    if Eco_Car_state(1) > Obstacle1_state(1) - CarSize(1) && Eco_Car_state(1) < Obstacle1_state(1) + CarSize(1)
        if Eco_Car_state(2) > Obstacle1_state(2) - CarSize(2) && Eco_Car_state(2) < Obstacle1_state(2) + CarSize(2)
            Flag_Collision = 1;
        end
    end
    if Eco_Car_state(1) > Obstacle2_state(1) - CarSize(1) && Eco_Car_state(1) < Obstacle2_state(1) + CarSize(1)
        if Eco_Car_state(2) > Obstacle2_state(2) - CarSize(2) && Eco_Car_state(2) < Obstacle2_state(2) + CarSize(2)
            Flag_Collision = 1;
        end
    end
%     if Eco_Car_state(1) > Obstacle3_state(1) - CarSize(1) && Eco_Car_state(1) < Obstacle3_state(1) + CarSize(1)
%         if Eco_Car_state(2) > Obstacle3_state(2) - CarSize(2) && Eco_Car_state(2) < Obstacle3_state(2) + CarSize(2)
%             Flag_Collision = 1;
%         end
%     end
    
    if Flag_Collision == 0
        rectangle('Position',[Eco_Car_state(1)-0.5*CarSize(1), Eco_Car_state(2)-0.5*CarSize(2), CarSize(1), CarSize(2)],'FaceColor','b');
    else
        rectangle('Position',[Eco_Car_state(1)-0.5*CarSize(1), Eco_Car_state(2)-0.5*CarSize(2), CarSize(1), CarSize(2)],'FaceColor','r');
%         pause(0.5);
    end  
    hold off
    
    figure(4)
    clf
    hold on
    plot(Sensor_Angle/10,ODG_Value(:,step),'k')
    plot(Sensor_Angle/10,ODG_Value_temp(:,step,1),'r')
    plot(Sensor_Angle/10,ODG_Value_temp(:,step,2),'g')
%     plot(Sensor_Angle/10,ODG_Value_temp(:,step,3),'b')
    hold off
    drawnow
    
    if Eco_Car_state(1) > 1000
        break; 
    end
    
end
%%









