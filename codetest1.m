%%test code

%disp(scanSub.LatestMessage.Ranges)
sample = 120;


%while 1
data_reverse=zeros(length(scanSub.LatestMessage.Ranges),1);
data_only=scanSub.LatestMessage.Ranges(:,1);

%[TK TH] =min(data_only);

Sensing_step=1;
Sensing_width = [0:Sensing_step:sample-1]';

%data.Data=TH;
%send (mpcPub,data);
        %temcost =0.0001;!1971
        !
       
%Noize=noizeSub.LatestMessage.Data;
%disp (data_only(:,1) <=2);

ture_data=data_only(:,1) <=2;

k=0;
pointdn = zeros(1,sample);
pointup = zeros(1,sample);

ture_data(1,1)=0;
ture_data(sample,1)=0;

dk_save=zeros(1,sample);
WH=0;
dk_max=12;
Sensor_Data = zeros(size(Sensing_width,1),2);
Sensor_Data(:,1) = Sensing_width;
cunt =0;
W_Robot= 0.2;


%Sensor_Data(:,2) = zeros(360,1);
    
for Step = 1 : sample-1

    
    if ture_data(Step,1) ~= ture_data(Step+1,1)
        k=k+1;
        
        if ture_data(Step+1,1) == 0
            pointdn(1,k)=Step;
        end
        
        if ture_data(Step+1,1) == 1
            pointup(1,k)=Step;
        end
             

    end
    rescan(Step,1)=data_only(Step,1);
    if data_only(Step,1) == inf
        rescan(Step) =12;
    end
    lescan(Step,1) =12-rescan(Step,1);
    lescan2(Step,1) =12-data_only(Step,1);
    oneline(Step,1)=2;
    
    
    
    
    
end

save=k/2;

odg_Sigma_size = zeros(1,save);
odg_center_size = zeros(1,save);
odg_min_d = zeros(1,save);
odg_mean_d = zeros(1,save);


for odg_pint =1:k
    
    if mod(odg_pint,2) ==1
       
       save=(odg_pint+1)/2
       odg_Sigma_size(1,save) =pointdn(1,odg_pint+1)-pointup(1,odg_pint)+1;
       odg_center_size(1,save) =(pointdn(1,odg_pint+1)+pointup(1,odg_pint))/2;
       
     
       
       dk_save(1,save)= data_only(round(odg_center_size(1,save)),1);
       %[TK TH]=min (data_on[8695126.0, 9160005.0, 9638058.0, 10128784.0, 10631610.0, 11145889.0, 11670900.0, 122058ly(pointdn(1,odg_pint+1),pointup(1,odg_pint)));
       %[TK TH]=min (data_only(pointup(1,odg_pint),pointdn(1,odg_pint+1)));
       %dk_save(1,odg_pint/2)= TH;
       
       
       
       find_center=data_only(pointup(odg_pint)+1:pointdn(1,odg_pint+1),1:1);
       [when va]= min (find_center);
       
       odg_min_d(1,save) =when;
       
       
       odg_mean_d(1,save) =mean(data_only(pointup(odg_pint)+1:pointdn(1,odg_pint+1),1:1));
       
    end
    
   
    
     
end
    
    





%save=1
%[TK TH] =min(data_only);
%dk = 

for odg_start =1 : save
    
  
    WH = odg_Sigma_size(1,odg_start);
    
    CH = odg_center_size(1,odg_start);
    CCH =CH-180;
    dk=dk_save(1,odg_start); % 
    dk_min =odg_min_d(1,odg_start);% min v
    %disp(odg_min_d);
    odg_mean_dk =odg_mean_d(1,odg_start);% mean v
    
    dkt=dk_max-dk_min; %A_k
    %SigmaL = 2*atan2(odg_mean_dk*tan(CH/2)+(W_Robot/2),odg_mean_dk);
    SigmaL=  2*sqrt(WH)*log(WH/10);
    if SigmaL==0
        SigmaL=1;
    end
    
   
    gaussmf(Sensor_Data(:,1),[SigmaL,CH])
    
    Sensor_Temp = (dkt*exp(1/2))*gaussmf(Sensor_Data(:,1),[SigmaL,CH]);
    
    
    Sensor_Data(:,2)= Sensor_Data(:,2)+Sensor_Temp;
    
end




%data.Data=TH;
%send (mpcPub,data);
%disp(save*2)
%save=save*2;
%Sensor_Data(:,2)=Sensor_Data(:,2)./save;
%kkks=Sensor_Data([125:245],2)


%%360
%[minva alg]= min (Sensor_Data([125:245],2));
%whens=find ( Sensor_Data(:,2)==minva) ;


%% 
[minva alg]= min (Sensor_Data(:,2));
whens=find ( Sensor_Data(:,2)==minva) ;

% A = Sensor_Data([10:30,50:60],1)
% A = Sensor_Data([10:2:60],1) %10 12 14 16

data.Ranges=Sensor_Data(:,2);
data.RangeMin= minva;

if whens ==0 
    data.AngleMin=sample/2;
else
    data.AngleMin= whens(1,1)-1;
end
send (ScanPub,data);



%Sensor_Data(:,2) = zeros(360,1);


%end


plot(Sensor_Data(:,end))
title('Sensor Data')
xlabel('angle') 
ylabel('distance in m') 

hold on
%plot(data_only)
hold on
%plot(oneline)
hold on
%plot(rescan)
hold on
plot (lescan)
legend ({'ODG(Obstacle Dependent Gaussian)','LiDAR'},'Location','southwest')

hold on
%plot (lescan2)
%end