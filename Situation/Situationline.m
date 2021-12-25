function PP_Simulation = Situationline()
PP_Simulation = struct();

Time_Step = 0.1;
Time = 100;
Left_Line = 5;
Right_Line = -5;
CarSize = [5; 2.5];

PP_Simulation.TimeStep = Time_Step;
PP_Simulation.Time = 0:PP_Simulation.TimeStep:Time;
PP_Simulation.SimulationStep = length(PP_Simulation.Time);

%% Eco_Car
Eco_Car = struct();
Eco_Car.Size = CarSize;
Eco_Car.x = 0;
Eco_Car.y = -2.5;
Eco_Car.vx = 20;
Eco_Car.vy = 0;
Eco_Car.RefV = 20;
PP_Simulation.Eco_Car = Eco_Car;
PP_Simulation.MinX = Eco_Car.x;
PP_Simulation.MaxX = Eco_Car.x;

%% Obstacle
Obstacle = struct();
PP_Simulation.Num_Obstacle = 0;


% Obstacle_Car 1 : Slow pront
Obstacle.Size = CarSize;
Obstacle.x = 60;
Obstacle.y = -2.5;
Obstacle.vx = 15;           %Math_Velocity_Converter(50,0);
Obstacle.vy = 0;
Obstacle.pos = zeros(4,length(PP_Simulation.Time));
Obstacle.pos(:,1) = [Obstacle.x;Obstacle.y;Obstacle.vx;Obstacle.vy];

for j = 2:length(PP_Simulation.Time)
    Obstacle.pos(:,j) = Dynamic_Model_1(Obstacle.pos(:,j-1),[0,0],PP_Simulation.TimeStep);
end
Min_X_Pos = min(Obstacle.pos(1,:));
Max_X_Pos = max(Obstacle.pos(1,:));
PP_Simulation.Num_Obstacle = PP_Simulation.Num_Obstacle + 1;
PP_Simulation.Obstacle(PP_Simulation.Num_Obstacle) = Obstacle;
if (PP_Simulation.MinX > Min_X_Pos); PP_Simulation.MinX = Min_X_Pos; end
if (PP_Simulation.MaxX < Max_X_Pos); PP_Simulation.MaxX = Max_X_Pos; end


% % Obstacle_Car 2 : Def line Fast
% Obstacle.Size = CarSize;
% Obstacle.x = -80;
% Obstacle.y = -2.5;
% % Obstacle.vx = Math_Velocity_Converter(40,0); 
% Obstacle.vx = Math_Velocity_Converter(70,0);
% Obstacle.vy = 0;
% Obstacle.pos = zeros(4,length(PP_Simulation.Time));
% Obstacle.pos(:,1) = [Obstacle.x;Obstacle.y;Obstacle.vx;Obstacle.vy];
% for j = 2:length(PP_Simulation.Time)
%     Obstacle.pos(:,j) = Dynamic_Model_1(Obstacle.pos(:,j-1),[0,0],PP_Simulation.TimeStep);
% end
% Min_X_Pos = min(Obstacle.pos(1,:));
% Max_X_Pos = max(Obstacle.pos(1,:));
% PP_Simulation.Num_Obstacle = PP_Simulation.Num_Obstacle + 1;
% PP_Simulation.Obstacle(PP_Simulation.Num_Obstacle) = Obstacle;
% if (PP_Simulation.MinX > Min_X_Pos); PP_Simulation.MinX = Min_X_Pos; end
% if (PP_Simulation.MaxX < Max_X_Pos); PP_Simulation.MaxX = Max_X_Pos; end


%% Lane
PP_Simulation.LLine = [PP_Simulation.MinX:PP_Simulation.MaxX; Left_Line *ones(1,length(PP_Simulation.MinX:PP_Simulation.MaxX))];
PP_Simulation.RLine = [PP_Simulation.MinX:PP_Simulation.MaxX; Right_Line*ones(1,length(PP_Simulation.MinX:PP_Simulation.MaxX))];

end