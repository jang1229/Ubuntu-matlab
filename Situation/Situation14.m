function PP_Simulation = Situation04()
PP_Simulation = struct();

Time_Step = 0.2;
Time = 100;
Left_Line = 6;
Right_Line = -6;
CarSize = [4.1; 1.7]; % European Van

PP_Simulation.TimeStep = Time_Step;
PP_Simulation.Time = 0:PP_Simulation.TimeStep:Time;
PP_Simulation.SimulationStep = length(PP_Simulation.Time);

%% Eco_Car
Eco_Car = struct();
Eco_Car.Size = CarSize;
Eco_Car.x = 0;
Eco_Car.y = 3;
Eco_Car.vx = Math_Velocity_Converter(70,0);
Eco_Car.vy = 0;
PP_Simulation.Eco_Car = Eco_Car;
PP_Simulation.MinX = Eco_Car.x;
PP_Simulation.MaxX = Eco_Car.x;

%% Obstacle
Obstacle = struct();
PP_Simulation.Num_Obstacle = 0;
% Obstacle_Car 1 : Cut-in
Obstacle.Size = CarSize;
Obstacle.x = 80;
Obstacle.y = 3;
Obstacle.vx = Math_Velocity_Converter(45,0);
Obstacle.vy = 0;
Obstacle.pos = zeros(4,length(PP_Simulation.Time));
Obstacle.pos(:,1) = [Obstacle.x;Obstacle.y;Obstacle.vx;Obstacle.vy];
ObstacleUTime = 110;
ObstacleU = [0, -0.6];
Flag = 0;
for j = 2:length(PP_Simulation.Time)
    U = [0,0];
    if j == ObstacleUTime
        U = ObstacleU;
        Flag = 1;
    end
    if Obstacle.pos(2,j-1) < -3 && Flag == 1
        U = -ObstacleU;
        Flag = 0;
    end
    
    Obstacle.pos(:,j) = Dynamic_Model_1(Obstacle.pos(:,j-1),U,PP_Simulation.TimeStep);
end
Min_X_Pos = min(Obstacle.pos(1,:));
Max_X_Pos = max(Obstacle.pos(1,:));
PP_Simulation.Num_Obstacle = PP_Simulation.Num_Obstacle + 1;
PP_Simulation.Obstacle(PP_Simulation.Num_Obstacle) = Obstacle;
if (PP_Simulation.MinX > Min_X_Pos); PP_Simulation.MinX = Min_X_Pos; end
if (PP_Simulation.MaxX < Max_X_Pos); PP_Simulation.MaxX = Max_X_Pos; end

% Obstacle_Car 2 : Cut-out
Obstacle.Size = CarSize;
Obstacle.x = 90;
Obstacle.y = -3;
Obstacle.vx = Math_Velocity_Converter(65,0);
Obstacle.vy = 0;
Obstacle.pos = zeros(4,length(PP_Simulation.Time));
Obstacle.pos(:,1) = [Obstacle.x;Obstacle.y;Obstacle.vx;Obstacle.vy];
ObstacleUTime = 310;
ObstacleU = [0, 0.6];
Flag = 0;
for j = 2:length(PP_Simulation.Time)
    U = [0,0];
    if j == ObstacleUTime
        U = ObstacleU;
        Flag = 1;
    end
    if Obstacle.pos(2,j-1) > 3 && Flag == 1
        U = -ObstacleU;
        Flag = 0;
    end
    
    Obstacle.pos(:,j) = Dynamic_Model_1(Obstacle.pos(:,j-1),U,PP_Simulation.TimeStep);
end
Min_X_Pos = min(Obstacle.pos(1,:));
Max_X_Pos = max(Obstacle.pos(1,:));
PP_Simulation.Num_Obstacle = PP_Simulation.Num_Obstacle + 1;
PP_Simulation.Obstacle(PP_Simulation.Num_Obstacle) = Obstacle;
if (PP_Simulation.MinX > Min_X_Pos); PP_Simulation.MinX = Min_X_Pos; end
if (PP_Simulation.MaxX < Max_X_Pos); PP_Simulation.MaxX = Max_X_Pos; end

% Obstacle_Car 3 : Cut-in
Obstacle.Size = CarSize;
Obstacle.x = 250;
Obstacle.y = 3;
Obstacle.vx = Math_Velocity_Converter(50,0);
Obstacle.vy = 0;
Obstacle.pos = zeros(4,length(PP_Simulation.Time));
Obstacle.pos(:,1) = [Obstacle.x;Obstacle.y;Obstacle.vx;Obstacle.vy];
ObstacleUTime = 510;
ObstacleU = [0, -0.6];
Flag = 0;
for j = 2:length(PP_Simulation.Time)
    U = [0,0];
    if j == ObstacleUTime
        U = ObstacleU;
        Flag = 1;
    end
    if Obstacle.pos(2,j-1) < -3 && Flag == 1
        U = -ObstacleU;
        Flag = 0;
    end
    
    Obstacle.pos(:,j) = Dynamic_Model_1(Obstacle.pos(:,j-1),U,PP_Simulation.TimeStep);
end
Min_X_Pos = min(Obstacle.pos(1,:));
Max_X_Pos = max(Obstacle.pos(1,:));
PP_Simulation.Num_Obstacle = PP_Simulation.Num_Obstacle + 1;
PP_Simulation.Obstacle(PP_Simulation.Num_Obstacle) = Obstacle;
if (PP_Simulation.MinX > Min_X_Pos); PP_Simulation.MinX = Min_X_Pos; end
if (PP_Simulation.MaxX < Max_X_Pos); PP_Simulation.MaxX = Max_X_Pos; end



PP_Simulation.LLine = [PP_Simulation.MinX:PP_Simulation.MaxX;Left_Line *ones(1,length(PP_Simulation.MinX:PP_Simulation.MaxX))];
PP_Simulation.RLine = [PP_Simulation.MinX:PP_Simulation.MaxX;Right_Line*ones(1,length(PP_Simulation.MinX:PP_Simulation.MaxX))];

% ShowSim(PP_Simulation)
end

