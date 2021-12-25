clc; clear; close all
FileName = 'Save_Result_Sim23_C.mat';

DS_Result = struct();
load(FileName)

% PP_Simulation = Situation01();
% PP_Simulation = SituationT1();
% PP_Simulation = Situation11();
% PP_Simulation = Situation12();
% PP_Simulation = Situation21();
% PP_Simulation = Situation22();
PP_Simulation = Situation23();


DS_Result.Sim = PP_Simulation;

% %% MPC N 50
Num_Data = 7;
Num_Data = Num_Data + 1;
Data_Name = 'MPC(N:50)';
% [Eco_Car_result, Eco_Car_result_MPC] = MPC_type1_50(PP_Simulation);
[Eco_Car_result, Eco_Car_result_MPC] = MPC_type2_50(PP_Simulation);
DS_Result.Result(Num_Data).Name = Data_Name;
DS_Result.Result(Num_Data).Result = Eco_Car_result;
DS_Result.Result(Num_Data).ResultShow= Eco_Car_result_MPC;
DS_Result.Result(Num_Data).Eval = Evaluation(DS_Result.Result(Num_Data).Result,DS_Result.Sim);


% Num_Data = 0;
% %% ODG N 15 R 80
% Num_Data = Num_Data + 1;
% Data_Name = 'ODGMPC(R:80)';
% [Eco_Car_result, Eco_Car_result_MPC] = ODGMPC_R80(PP_Simulation);
% DS_Result.Result(Num_Data).Name = Data_Name;
% DS_Result.Result(Num_Data).Result = Eco_Car_result;
% DS_Result.Result(Num_Data).ResultShow= Eco_Car_result_MPC;
% DS_Result.Result(Num_Data).Eval = Evaluation(DS_Result.Result(Num_Data).Result,DS_Result.Sim);

%% ODG N 10 R 85
% Num_Data = Num_Data + 1;
% Data_Name = 'ODGMPC(R:85)';
% [Eco_Car_result, Eco_Car_result_MPC] = ODGMPC_R85(PP_Simulation);
% DS_Result.Result(Num_Data).Name = Data_Name;
% DS_Result.Result(Num_Data).Result = Eco_Car_result;
% DS_Result.Result(Num_Data).ResultShow= Eco_Car_result_MPC;
% DS_Result.Result(Num_Data).Eval = Evaluation(DS_Result.Result(Num_Data).Result,DS_Result.Sim);
% 
% %% ODG N 10 R 90
% Num_Data = Num_Data + 1;
% Data_Name = 'ODGMPC(R:90)';
% [Eco_Car_result, Eco_Car_result_MPC] = ODGMPC_R90(PP_Simulation);
% DS_Result.Result(Num_Data).Name = Data_Name;
% DS_Result.Result(Num_Data).Result = Eco_Car_result;
% DS_Result.Result(Num_Data).ResultShow= Eco_Car_result_MPC;
% DS_Result.Result(Num_Data).Eval = Evaluation(DS_Result.Result(Num_Data).Result,DS_Result.Sim);
% 
% %% ODG N 10 R 95
% Num_Data = Num_Data + 1;
% Data_Name = 'ODGMPC(R:95)';
% [Eco_Car_result, Eco_Car_result_MPC] = ODGMPC_R95(PP_Simulation);
% DS_Result.Result(Num_Data).Name = Data_Name;
% DS_Result.Result(Num_Data).Result = Eco_Car_result;
% DS_Result.Result(Num_Data).ResultShow= Eco_Car_result_MPC;
% DS_Result.Result(Num_Data).Eval = Evaluation(DS_Result.Result(Num_Data).Result,DS_Result.Sim);
% 
% %% ODG N 10 R 99
% Num_Data = Num_Data + 1;
% Data_Name = 'ODGMPC(R:99)';
% [Eco_Car_result, Eco_Car_result_MPC] = ODGMPC_R99(PP_Simulation);
% DS_Result.Result(Num_Data).Name = Data_Name;
% DS_Result.Result(Num_Data).Result = Eco_Car_result;
% DS_Result.Result(Num_Data).ResultShow= Eco_Car_result_MPC;
% DS_Result.Result(Num_Data).Eval = Evaluation(DS_Result.Result(Num_Data).Result,DS_Result.Sim);
% 
% %% Compare 1 RHM type 1 N 10
% Num_Data = Num_Data + 1;
% Data_Name = 'Compare1_RHM(N:15)';
% [Eco_Car_result, Eco_Car_result_MPC] = Compare1_RHM_Type1_10(PP_Simulation);
% DS_Result.Result(Num_Data).Name = Data_Name;
% DS_Result.Result(Num_Data).Result = Eco_Car_result;
% DS_Result.Result(Num_Data).ResultShow= Eco_Car_result_MPC;
% DS_Result.Result(Num_Data).Eval = Evaluation(DS_Result.Result(Num_Data).Result,DS_Result.Sim);
% 
% %% Compare 1 RHM type 1 N 50
% Num_Data = Num_Data + 1;
% Data_Name = 'Compare1_RHM(N:50)';
% [Eco_Car_result, Eco_Car_result_MPC] = Compare1_RHM_Type1_50(PP_Simulation);
% DS_Result.Result(Num_Data).Name = Data_Name;
% DS_Result.Result(Num_Data).Result = Eco_Car_result;
% DS_Result.Result(Num_Data).ResultShow= Eco_Car_result_MPC;
% DS_Result.Result(Num_Data).Eval = Evaluation(DS_Result.Result(Num_Data).Result,DS_Result.Sim);




save(FileName,'DS_Result');

return
% default
Sigma = 0.5;
w1 = 0.25;
SigmaC = 1.5;
Risk_Trans = 10;

Sigma_Test = [0.1,0.45,1];
for i = 2 : 2
    Sigma = Sigma_Test(i);
    Test_1903112_ver1
    DS_Result(num).result = Eco_Car_result;
    num = num + 1;
end
Sigma = 0.5;


% w1_Test = [0.8,0.5,0.25];
% for i = 1 : 3
%     w1 = w1_Test(i);
%     Test_1903112_ver1
%     DS_Result(num).result = Eco_Car_result;
%     num = num + 1;
% end
% w1 = 0.25;
% 
% Risk_Trans_Test = [1,10,50];
% for i = 1 : 3
%     Risk_Trans = Risk_Trans_Test(i);
%     Test_1903112_ver1
%     DS_Result(num).result = Eco_Car_result;
%     num = num + 1
% end
% Risk_Trans = 10;
% 
% SigmaC_Test = [0.5,1.5,2];
% for i = 1 : 3
%     SigmaC = SigmaC_Test(i);
%     Test_1903112_ver1
%     DS_Result(num).result = Eco_Car_result;
%     num = num + 1
% end
% SigmaC=1.5;

