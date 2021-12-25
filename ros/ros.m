
rosinit; % ros 시작 코드  
addpath (genpath ( 'C:\ProgramData\MATLAB\SupportPackages\R2020a\')) % rostol box 경로 추가
addpath (genpath ('C:\Users\JSH\OneDrive\문서\MATLAB\Examples\R2020a\ros\GenerateAStandaloneROSNodeFromSimulinkExample' ))
% rosnode list %ros 모든 노드 목록 보기 

rosshutdown % ros 모든 연결 해지


%외부 ros 마스터에 연결하나는 방법 

setenv ( 'ROS_IP' , '192.168.0.14:22' )
setenv ( 'ROS_MASTER_URI' , 'http://192.168.0.15:11311' )
setenv ( 'ROS_HOSTNAME' , 'jetson' )

%변수 확인 
getenv ( 'ROS_MASTER_URI' )
getenv ( 'ROS_HOSTNAME' )
getenv ( 'ROS_IP' )



%%  set up 
rosinit  %% ros 시작 
%master = ros.Core;

%mySub =rossubscriber ('/sub_topic');

%% node 생성 방법 
% node_1 = ros.Node('node_1', masterHost);
% 노드이름변수 = ros.Node ('노드이름', 호스트 주소 );
%%
node5 = ros.Node('/test_node_1');
node6 = ros.Node('/test_node_2');

pub = ros.Publisher(node5,'/chatter3','std_msgs/String'); 
% 이름 = ros pub (선언된 노드 , topic 이름 . 토픽 구조 형식)
sub = ros.Subscriber(node6,'/chatter3','std_msgs/String');


msg = rosmessage('std_msgs/String');
msg.Data = 'Message from Node 1';

send(pub,sub)

%% serviceserver 방법 

% rosservice 서비스에대한 정보검색 
% 형식 
%rosservice list
%rosservice info svcname   정보  ex  svcinfo = rosservice ( 'info' , 'gazebo / pause_physics' )
%rosservice type svcname   유형
%rosservice uri svcname   URI IP 주소 

%함수정리 rossvcserver =   서비스 서버 생성하는 함수  
%형식 

%server = rossvcserver(servicename,svctype)   
%server = rossvcserver(servicename,svctype,callback)    (서버 이름 , 타입 ,콜백 함수 지정 Callback 형식은 NewM)
%server = ros.ServiceServer(node, name,type)
%server = ros.ServiceServer(node, name,type,callback)
testserver = rossvcserver('/what',   'std_srvs/Empty' , @test_fun);










%% topic 만드는방법 


chatterpub = rospublisher('/chatter', 'std_msgs/String') %%  chatterpub 변수 = rospublisher함수 (topic name , topic 구조 )
pause(2) % Wait to ensure publisher is registered  % 등록하기위한 약간의 시간 

chattermsg = rosmessage(chatterpub);
chattermsg.Data = 'hello world'

rostopic list %  topic name chatter 확인 

%% topic 연결 확인
send(chatterpub,chattermsg)
pause(2)




%%  ex 파일 
%node 생성 방법 
masterHost =  'http://192.168.0.15:11311';
node_1 = ros.Node('node_1', masterHost);
node_2 = ros.Node('node_2', masterHost);
node_3 = ros.Node('node_3', masterHost);

% Create a publisher and subscriber for the '/pose' topic
twistPub = ros.Publisher(node_1,'/pose','geometry_msgs/Twist');
twistPubmsg = rosmessage(twistPub);
twistSub = ros.Subscriber(node_2,'/pose');

% Create publishers and subscribers for the '/scan' topic
scanPub = ros.Publisher(node_3,'/scan','sensor_msgs/LaserScan');
scanPubmsg = exampleHelperROSLoadRanges();
scanSub1 = ros.Subscriber(node_1,'/scan');
scanSub2 = ros.Subscriber(node_2,'/scan');

% Create two service servers for the '/add' and '/reply' services
srv1 = ros.ServiceServer(node_3,'/add', 'roscpp_tutorials/TwoInts');
srv2 = ros.ServiceServer(node_3,'/reply', 'std_srvs/Empty', @exampleHelperROSEmptyCallback);

% Load sample data for inspecting messages
tffile = fullfile(fileparts(mfilename('fullpath')), '..', 'data', 'tfdata.mat');
tfcell = load(tffile);
tf = tfcell.t;
clear tffile
clear tfcell

% Create a timer for publishing messages and assign appropriate handles
% The timer will call exampleHelperROSSimTimer at a rate of 10 Hz.
timerHandles.twistPub = twistPub;
timerHandles.twistPubmsg = twistPubmsg;
timerHandles.scanPub = scanPub;
timerHandles.scanPubmsg = scanPubmsg;
simTimer = ExampleHelperROSTimer(0.1, {@exampleHelperROSSimTimer,timerHandles});
%%