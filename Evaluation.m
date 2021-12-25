function Eval = Evaluation(Path_Data, Map_Data)

Eval = struct();

%% Base Data
Eval.Abs_Path = Path_Data(1:2,:);
Eval.Abs_Vel = Path_Data(3:4,:);
Eval.Abs_Acc = (Path_Data(3:4,2:end) - Path_Data(3:4,1:end-1))./Map_Data.TimeStep;

%% D_O
for k = 1 : Map_Data.Num_Obstacle
    Eval.Obs(k).pos = Map_Data.Obstacle(k).pos(1:2,:) - Path_Data(1:2,:);
end

% 정량적 지표
%% Path Cost
Path_Line = 0;
Path_Angle = 0;
Path_Angle_add = zeros(size(Path_Data,2)-1,1);
for k = 1 : size(Path_Data,2)-1
    Path_Line = Path_Line + max(abs(Path_Data(2,k)+2.5),abs(Path_Data(2,k)))/5;
  
    Path_Angle_add(k) = abs(atan2d(Path_Data(2,k+1) - Path_Data(2,k),Path_Data(1,k+1) - Path_Data(1,k)));
    if Path_Angle_add(k) > 90; Path_Angle_add(k) = 180-Path_Angle_add(k); end
    Path_Angle = Path_Angle + Path_Angle_add(k)/180;
end

Path_Line = Path_Line/k;
Path_Angle = Path_Angle/k;
Eval.FR = Path_Angle;
Eval.DR = Path_Line;
Eval.ST = Eval.DR * (1-Eval.FR);



%% 

end













