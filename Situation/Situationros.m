function PP_Simulation = Situationros()
PP_Simulation = struct();

Time_Step = 0.1;
Time = 1000;
CarSize = [10; 10];
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






end