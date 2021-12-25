function Next_State = Dynamic_Model_1(State,Input,dt)
    Next_State = zeros(size(State));
    Next_State(1) = State(1) + dt*State(3);
    Next_State(2) = State(2) + dt*State(4);
    Next_State(3) = State(3) + dt*Input(1);
    Next_State(4) = State(4) + dt*Input(2);
end