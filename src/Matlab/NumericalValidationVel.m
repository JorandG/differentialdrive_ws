function NumericalValidation(ReAll, humanData, u)
global num_agents indexVal indexValW FirstReAll

if humanData{u}.RobotVelocityProximity(humanData{u}.Task-1) < 0 % human asks for a slower robot
    taup = ReAll.timeS(num_humans*num_filling_boxes*3+u+(humanData{u}.Task)) - ReAll.timeS(num_humans*num_filling_boxes*4+u+(humanData{u}.Task)) %ReAll.timeSh(u+num_agents*humanData{u}.Task) - ReAll.timeFh(u+num_agents)
    taup1 = ReAll.timeS(num_humans*num_filling_boxes*3+u+(humanData{u}.Task-1)) - ReAll.timeS(num_humans*num_filling_boxes*4+u+(humanData{u}.Task)) %ReAll.timeSh(u+num_agents*humanData{u}.Task-1) - ReAll.timeFh(u)
    if taup > taup1
        indexVal{u} = indexVal{u} - (1/humanData{u}.RobotVelocityProximity(humanData{u}.Task-1))*(abs(taup - taup1)/taup)
    elseif taup < taup1
        indexVal{u} = indexVal{u} + (1/humanData{u}.RobotVelocityProximity(humanData{u}.Task-1))*(abs(taup - taup1)/taup)
    elseif taup == taup1
        indexVal{u} = indexVal{u}
    end
end

if humanData{u}.RobotVelocityProximity(humanData{u}.Task-1) > 0 % human asks for a faster robot
    taup = ReAll.timeS(num_humans*num_filling_boxes*3+u+(humanData{u}.Task)) - ReAll.timeS(num_humans*num_filling_boxes*4+u+(humanData{u}.Task)) %ReAll.timeSh(u+num_agents*humanData{u}.Task) - ReAll.timeFh(u+num_agents)
    taup1 = ReAll.timeS(num_humans*num_filling_boxes*3+u+(humanData{u}.Task-1)) - ReAll.timeS(num_humans*num_filling_boxes*4+u+(humanData{u}.Task))
    if taup > taup1
        indexVal{u} = indexVal{u} - (1/humanData{u}.RobotVelocityProximity(humanData{u}.Task-1))*(abs(taup - taup1)/taup)
    elseif taup < taup1
        indexVal{u} = indexVal{u} + (1/humanData{u}.RobotVelocityProximity(humanData{u}.Task-1))*(abs(taup - taup1)/taup)
    elseif taup == taup1
        indexVal{u} = indexVal{u}
    end
end

%% Without ReAll
if humanData{u}.RobotVelocityProximity(humanData{u}.Task-1) < 0 % human asks for a slower robot
    taup = FirstReAll.timeS(num_humans*num_filling_boxes*3+u+(humanData{u}.Task)) - FirstReAll.timeS(num_humans*num_filling_boxes*4+u+(humanData{u}.Task)) %ReAll.timeSh(u+num_agents*humanData{u}.Task) - ReAll.timeFh(u+num_agents)
    taup1 = FirstReAll.timeS(num_humans*num_filling_boxes*3+u+(humanData{u}.Task-1)) - FirstReAll.timeS(num_humans*num_filling_boxes*4+u+(humanData{u}.Task)) %ReAll.timeSh(u+num_agents*humanData{u}.Task-1) - ReAll.timeFh(u)
    if taup > taup1
        indexVal{u} = indexVal{u} - (1/humanData{u}.RobotVelocityProximity(humanData{u}.Task-1))*(abs(taup - taup1)/taup)
    elseif taup < taup1
        indexVal{u} = indexVal{u} + (1/humanData{u}.RobotVelocityProximity(humanData{u}.Task-1))*(abs(taup - taup1)/taup)
    elseif taup == taup1
        indexVal{u} = indexVal{u}
    end
end

if humanData{u}.RobotVelocityProximity(humanData{u}.Task-1) > 0 % human asks for a faster robot
    taup = FirstReAll.timeS(num_humans*num_filling_boxes*3+u+(humanData{u}.Task)) - FirstReAll.timeS(num_humans*num_filling_boxes*4+u+(humanData{u}.Task)) %ReAll.timeSh(u+num_agents*humanData{u}.Task) - ReAll.timeFh(u+num_agents)
    taup1 = FirstReAll.timeS(num_humans*num_filling_boxes*3+u+(humanData{u}.Task-1)) - FirstReAll.timeS(num_humans*num_filling_boxes*4+u+(humanData{u}.Task))
    if taup > taup1
        indexVal{u} = indexVal{u} - (1/humanData{u}.RobotVelocityProximity(humanData{u}.Task-1))*(abs(taup - taup1)/taup)
    elseif taup < taup1
        indexVal{u} = indexVal{u} + (1/humanData{u}.RobotVelocityProximity(humanData{u}.Task-1))*(abs(taup - taup1)/taup)
    elseif taup == taup1
        indexVal{u} = indexVal{u}
    end
end

indexVal
indexValW