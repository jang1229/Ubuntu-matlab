figure(3)
clf
Obs_Color = ['r','g','b'];
ODG_Value2=(ODG_Value)';

for step = 1 : PP_Simulation.SimulationStep
    Eco_Car_state = Eco_Car_result(:,step);
            
    % Default situation
    figure(3)
%     subplot(2,1,2)
   contour(ODG_Value2);
   % plot(PP_Simulation.LLine(2,:),PP_Simulation.LLine(1,:),'k','Linewidth',1.5);
    %hold on
    %plot(PP_Simulation.RLine(2,:),PP_Simulation.RLine(1,:),'k','Linewidth',1.5);
    
    %plot((PP_Simulation.LLine(2,:)+PP_Simulation.RLine(2,:))./2,PP_Simulation.LLine(1,:),'--k','Linewidth',1.5);
   %
    %plot(PP_Simulation5.LLine(2,:),PP_Simulation5.LLine(1,:),'k','Linewidth',1.5);
    %hold on
    %plot(PP_Simulation5.RLine(2,:),PP_Simulation5.RLine(1,:),'k','Linewidth',1.5);
    
    %plot((PP_Simulation5.LLine(2,:)+PP_Simulation5.RLine(2,:))./2,PP_Simulation5.LLine(1,:),'--k','Linewidth',1.5);
   % ylim([Eco_Car_state(1)-20, Eco_Car_state(1)+60])
    ylim([Eco_Car_state(1)-60, Eco_Car_state(1)+180])
   
    xlim([-6, 6])
    axis equal
    grid on      
     rectangle('Position',[Eco_Car_state(2)-0.5*PP_Simulation.Eco_Car.Size(2), Eco_Car_state(1)-0.5*PP_Simulation.Eco_Car.Size(1), PP_Simulation.Eco_Car.Size(2), PP_Simulation.Eco_Car.Size(1)]);
    
    Flag_Collision = 0;

    if Flag_Collision == 0
        rectangle('Position',[Eco_Car_state(2)-0.5*PP_Simulation.Eco_Car.Size(2), Eco_Car_state(1)-0.5*PP_Simulation.Eco_Car.Size(1), PP_Simulation.Eco_Car.Size(2), PP_Simulation.Eco_Car.Size(1)],'FaceColor','b');
    else
        rectangle('Position',[Eco_Car_state(2)-0.5*PP_Simulation.Eco_Car.Size(2), Eco_Car_state(1)-0.5*PP_Simulation.Eco_Car.Size(1), PP_Simulation.Eco_Car.Size(2), PP_Simulation.Eco_Car.Size(1)],'FaceColor','r');
    end  
    hold off
end
