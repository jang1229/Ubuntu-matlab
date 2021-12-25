
masterHost =  'http://192.168.0.14:11311';

scan_data = ros.Node('scan_data', masterHost);


node_2 = ros.Node('node_2', masterHost);
node_3 = ros.Node('node_3', masterHost);
%odg_1 = ros.Node('odg_1', masterHost);
%odg_2 = ros.Node('odg_2', masterHost);
%odg_3 = ros.Node('odg_3', masterHost);
odg_4 = ros.Node('node_odg', masterHost);
odg_5 = ros.Node('odg_5', masterHost);
path = ros.Node('path', masterHost);

scanSub= ros.Subscriber(node_2,'/scan','sensor_msgs/LaserScan'); %% 
%pathSub= ros.Subscriber(path,'/pf/pose/odom ','nav_msgs/Odometry'); %% 
noizeSub =ros.Subscriber(odg_5,'/odg_center','std_msgs/Float32MultiArray');
%mpcSub= ros.Subscriber(node_1,'/odg_data','std_msgs/Float32MultiArray'); %% 
mpcPub = ros.Publisher(node_3,'/noise_data','std_msgs/Int32MultiArray');%% 

ScanPub = ros.Publisher(odg_4,'/S_ODG_DATA','sensor_msgs/LaserScan');%% 

%anlgePub = ros.Publisher(node_2,'/anlge_data','std_msgs/Int32MultiArray');%% 
data=rosmessage(mpcPub); %% 
data=rosmessage(ScanPub); %% 
%data=rosmessage(anlgePub) %% 
%
%odg_s1 = ros.Subscriber(odg_1,'/odg_one','std_msgs/Float32MultiArray'); %% 
%odg_s2 = ros.Subscriber(odg_2,'/odg_two','std_msgs/Float32MultiArray'); %% 
%odg_s3 = ros.Subscriber(odg_3,'/odg_tr','std_msgs/Float32MultiArray'); %%  
%odg_s4 = ros.Subscriber(odg_4,'/odg_f','std_msgs/Float32MultiArray'); %%
%odg_s5 = ros.Subscriber(odg_5,'/odg_center','std_msgs/Float32MultiArray'); %% 
%odg_s6 = ros.Subscriber(odg_6,'/odg_data_xy','std_msgs/Int32MultiArray'); 
%rosshutdown

%mpctest2 = ros.Subscriber(node_2,'/mpc_data','std_msgs/Int32MultiArray'); %% 
%odg_xy_Sub = ros.Subscriber(node_3,'/odg_data_xy','std_msgs/Int32MultiArray'); %%