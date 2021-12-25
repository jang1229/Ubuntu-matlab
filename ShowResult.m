figure(3)
clf
Obs_Color = ['r','g','b'];
for step = 1 : PP_Simulation.SimulationStep
    Eco_Car_state = Eco_Car_result(:,step);
            
    % Default situation
    figure(3)
%     subplot(2,1,2)
    
    plot(PP_Simulation.LLine(2,:),PP_Simulation.LLine(1,:),'k','Linewidth',1.5);
    hold on
    plot(PP_Simulation.RLine(2,:),PP_Simulation.RLine(1,:),'k','Linewidth',1.5);
    plot((PP_Simulation.LLine(2,:)+PP_Simulation.RLine(2,:))./2,PP_Simulation.LLine(1,:),'--k','Linewidth',1.5);
    ylim([Eco_Car_state(1)-20, Eco_Car_state(1)+60])
    xlim([-6, 6])
    axis equal
    grid on      
    rectangle('Position',[Eco_Car_state(2)-0.5*PP_Simulation.Eco_Car.Size(2), Eco_Car_state(1)-0.5*PP_Simulation.Eco_Car.Size(1), PP_Simulation.Eco_Car.Size(2), PP_Simulation.Eco_Car.Size(1)]);
    
    Flag_Collision = 0;
    for k = 1 : PP_Simulation.Num_Obstacle
        % Get Info
        Obstacle_state = PP_Simulation.Obstacle(k).pos(:,step);
        % draw
        rectangle('Position',[Obstacle_state(2)-0.5*PP_Simulation.Obstacle(k).Size(2), Obstacle_state(1)-0.5*PP_Simulation.Obstacle(k).Size(1), PP_Simulation.Obstacle(k).Size(2), PP_Simulation.Obstacle(k).Size(1)],'FaceColor','g');
        % Collision Detection    
        if Eco_Car_state(1) > Obstacle_state(1) - PP_Simulation.Obstacle(k).Size(1) && Eco_Car_state(1) < Obstacle_state(1) + PP_Simulation.Obstacle(k).Size(1)
            if Eco_Car_state(2) > Obstacle_state(2) - PP_Simulation.Obstacle(k).Size(2) && Eco_Car_state(2) < Obstacle_state(2) + PP_Simulation.Obstacle(k).Size(2)
                Flag_Collision = 1;
            end
        end
    end
    if Flag_Collision == 0
        rectangle('Position',[Eco_Car_state(2)-0.5*PP_Simulation.Eco_Car.Size(2), Eco_Car_state(1)-0.5*PP_Simulation.Eco_Car.Size(1), PP_Simulation.Eco_Car.Size(2), PP_Simulation.Eco_Car.Size(1)],'FaceColor','b');
    else
        rectangle('Position',[Eco_Car_state(2)-0.5*PP_Simulation.Eco_Car.Size(2), Eco_Car_state(1)-0.5*PP_Simulation.Eco_Car.Size(1), PP_Simulation.Eco_Car.Size(2), PP_Simulation.Eco_Car.Size(1)],'FaceColor','r');
    end  
    hold off
    
%     figure(4)
%     plot(Sensor_Data(:,1),ODG_Value(:,step),'k')
%     hold on
%     for k = 1 : PP_Simulation.Num_Obstacle
%         plot(Sensor_Data(:,1),ODG_Value_temp(:,step,k),Obs_Color(k))
%     end
%     hold off
    drawnow
%     pause(0.05)
%     if Eco_Car_state(1) > 1000
%         break; 
%     end
    
%     if step == 74
%         pause()
%     end
    
   









end