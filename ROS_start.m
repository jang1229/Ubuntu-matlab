%% ros part 
PP_Simulation5 = Situation11(); 
setenv ( 'ROS_IP' , '192.168.0.14' )
setenv ( 'ROS_MASTER_URI' , 'http://192.168.0.15:11311' )
rosinit; % ros 시작 코드  
masterHost =  'http://192.168.0.15:11311';
node_1 = ros.Node('node_1', masterHost);
node_2 = ros.Node('node_2', masterHost);
node_3 = ros.Node('node_3', masterHost);
odg_1 = ros.Node('odg_1', masterHost);
odg_2 = ros.Node('odg_2', masterHost);
odg_3 = ros.Node('odg_3', masterHost);
odg_4 = ros.Node('odg_4', masterHost);
odg_5 = ros.Node('odg_5', masterHost);

mpcSub = ros.Subscriber(node_1,'/odg_data','std_msgs/Float32MultiArray'); %% 노드 연결 
mpcPub = ros.Publisher(node_1,'/mpc_data','std_msgs/Int32MultiArray');%% node_2에대 한 mpc_data 토픽생성 
data=rosmessage(mpcPub) %% 메세지 생성 
data=rosmessage(mpcPub) %% 메세지 생성 
%
odg_s1 = ros.Subscriber(odg_1,'/odg_one','std_msgs/Float32MultiArray'); %% 노드 연결 
odg_s2 = ros.Subscriber(odg_2,'/odg_two','std_msgs/Float32MultiArray'); %% 노드 연결 
odg_s3 = ros.Subscriber(odg_3,'/odg_tr','std_msgs/Float32MultiArray'); %% 노드 연결 
odg_s4 = ros.Subscriber(odg_4,'/odg_f','std_msgs/Float32MultiArray'); %% 노드 연결 
odg_s5 = ros.Subscriber(odg_4,'/odg_center','std_msgs/Float32MultiArray'); %% 노드 연결 

%

mpctest2 = ros.Subscriber(node_2,'/mpc_data','std_msgs/Int32MultiArray'); %% 노드 연결 
odg_xy_Sub = ros.Subscriber(node_3,'/odg_data_xy','std_msgs/Int32MultiArray'); %% 노드 연결 