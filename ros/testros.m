% exampleHelperROSCreateSampleNetwork

chatterpub = rospublisher('/chatter', 'std_msgs/String')  %% ros chatter노드에 구독자 하나 생성 스트림형으로 

pause(2) % Wait to ensure publisher is registered
chattermsg = rosmessage(chatterpub);  % ROS 메시지 생성
chattermsg.Data = 'hello world'
rostopic list
chattersub = rossubscriber('/chatter', @exampleHelperROSChatterCallback) %% %%  @exampleHelperROSChatterCallback == global_node 


send(chatterpub,chattermsg)

disp (chattersub)

pause(2)


chatterpub = rospublisher('/sc_two', 'std_msgs/String')  %% ros chatter노드에 구독자 하나 생성 스트림형으로 


%% ros code 순서 
rosinit; % ros 시작 코드  

rosshutdown % ros 모든 연결 해지


setenv ( 'ROS_IP' , '192.168.0.14' )
setenv ( 'ROS_MASTER_URI' , 'http://192.168.0.15:11311' )
rosinit; % ros 시작 코드  
masterHost =  'http://192.168.0.15:11311';

node_1 = ros.Node('node_1', masterHost);
node_2 = ros.Node('node_2', masterHost);
node_3 = ros.Node('node_3', masterHost);

mpcSub = ros.Subscriber(node_1,'/odg_data','std_msgs/String'); %% 노드 연결 
mpcPub = ros.Publisher(node_1,'/mpc_data','std_msgs/String');%% node_2에대 한 mpc_data 토픽생성 
data=rosmessage(mpcPub) %% 메세지 생성 
data.Data = "what";
% data.Data = "here";
send (mpcPub,data)

% 데이터 수집 및 데이터 변환 과정 
data_zero= zeros(5,160);
odg_data=mpcSub.LatestMessage.Data; %% 내부 Message 정보확인 
data_v=strsplit(odg_data) % string 형 공백으로 분할하기 
%mat = reshape (odg_data,[5,1108]) % 원하는 형식으로 변형 






%%rosmsg("list")msg 리스트 목록 
%%
twistPub = ros.Publisher(node_1,'/counter','std_msgs/String');
twistPubmsg = rosmessage(twistPub);
twistPubmsg.Data = 'hello world'

cointerSub = ros.Subscriber(node_2,'/odg_data',@test_fun);  %% topic에 노드연결하고 데이터 콜백받기  1회실시 
mpcSub = ros.Subscriber(node_1,'/odg_data'); %% 노드 연결 
testSub = ros.Subscriber(node_1,'/odg_data',@test_fun); %% 노드 연결 
twistSub = ros.Subscriber(node_3,'/camera/image',@test_fun);  %% 카메라 노드에 결과 값 저장  1회실시 

mpcPub = ros.Publisher(node_2,'/mpc_data','std_msgs/String');%% node_2에대 한 mpc_data 토픽생성 

mpcpub1 = rospublisher('/mpc_data', 'std_msgs/String') %% mpc_data 글로벌 노드 사용  토픽생성 

%% 순서대로 해야함 
mpcSub = ros.Subscriber(node_1,'/odg_data','std_msgs/String'); %% 노드 연결 
mpcPub = ros.Publisher(node_1,'/mpc_data','std_msgs/String');%% node_2에대 한 mpc_data 토픽생성 
data=rosmessage(mpcPub) %% 메세지 생성 
data.Data = "what";
data.Data = "here";
send (mpcPub,data)
mpctest2 = ros.Subscriber(node_2,'/mpc_data','std_msgs/String'); %% 노드 연결 
mpctest3 = ros.Subscriber(node_3,'/mpc_data','std_msgs/String'); %% 노드 연결 
mpcSub1.LatestMessage %% 내부 Message 정보확인 
mpcSub.LatestMessage.Data %% 내부 Message 정보확인 

rostopic echo /mpc_data % topic 데이터값 계속보기 

srv1 = ros.ServiceServer(node_3,'/add', 'roscpp_tutorials/TwoInts');
srv2 = ros.ServiceServer(node_3,'/reply', 'std_srvs/Empty', @exampleHelperROSEmptyCallback);

%%
chatpub = rospublisher('/mpc_data','std_msgs/String');
msg = rosmessage(chatpub);
msg.Data = 'test phrase';
send(chatpub,msg);

mpcSub1.LatestMessage.Data= mpcSub.LatestMessage.Data
%%
send(mpcpub,posedata)
msg = rosmessage(mpcpub);
msg.Data = posedata.Data;
send(twistPub,twistPubmsg);

twistSub = rossubscriber('/counter',node_1); 
%% topic 데이터 저장하는 방법 (ok 됨)
posesub = rossubscriber('/odg_data')
data = receive(posesub,10)
save('posedata.mat','data')
disp(data.Data)
%clear posedata %% 데이터 청소 
% delete('posedata.mat') %% 함수 청소 



%%

%% 영상topic 데이터 수집 
emptyimg = rosmessage('/camera/image')


%%


subscriber = ros.subscriber('/counter', 'std_msgs/String', 10, node_3);
subscriber.setOnNewMessageListeners({@test_fun});
pause(10);
node.removeSubscriber(subscriber);

twistSub.setOnNewMessageListeners({@test_fun})
 Sub  = Node.addSubscriber('/counter','std_msgs/String',10);
 Sub.setOnNewMessageListeners({@test_fun}
 
%%

% Create publishers and subscribers for the '/scan' topic
scanPub = ros.Publisher(node_3,'/scan','sensor_msgs/LaserScan');
scanPubmsg = exampleHelperROSLoadRanges();
scanSub1 = ros.Subscriber(node_1,'/scan');
scanSub2 = ros.Subscriber(node_2,'/scan');


%%
% Create two service servers for the '/add' and '/reply' services
srv1 = ros.ServiceServer(node_3,'/add', 'roscpp_tutorials/TwoInts');
srv2 = ros.ServiceServer(node_3,'/reply', 'std_srvs/Empty', @exampleHelperROSEmptyCallback);


%%
% Load sample data for inspecting messages
tffile = fullfile(fileparts(mfilename('fullpath')), '..', 'data', 'tfdata.mat');
tfcell = load(tffile);
tf = tfcell.t;
clear tffile
clear tfcell
%%
% Create a timer for publishing messages and assign appropriate handles
% The timer will call exampleHelperROSSimTimer at a rate of 10 Hz.
timerHandles.twistPub = twistPub;
timerHandles.twistPubmsg = twistPubmsg;
timerHandles.scanPub = scanPub;
timerHandles.scanPubmsg = scanPubmsg;
simTimer = ExampleHelperROSTimer(0.1, {@exampleHelperROSSimTimer,timerHandles});


%exampleHelperROSShutDownSampleNetwork
