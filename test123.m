figure(3)
%clf
Obs_Color = ['r','g','b'];
ODG_Value2=(ODG_Value)';

ODG_Value3=fliplr(ODG_Value2);

for step = 1 : 1000
    Eco_Car_state2 = Eco_Car_result(:,step);
    Eco_Car_state2(2) = Eco_Car_state2(2) +160;
    Eco_Car_result2(2,step)=Eco_Car_state2(2);
    
end

figure(3)
contour(ODG_Value);
hold on

%plot(atan2( Eco_Car_result(4,:), Eco_Car_result(3,:)))
%hold on

plot(Eco_Car_result2(2,:))




