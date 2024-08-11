function NumericalValidation(ReAll, humanData, u)
global num_agents indexVal indexValW FirstReAll

if humanData{u}.WaitingTime(humanData{u}.Task-1) < 0 % human asks for a longer waiting time
    taup = ReAll.timeSh(u+num_agents*humanData{u}.Task) - ReAll.timeFh(u+num_agents)
    taup1 = ReAll.timeSh(u+num_agents*humanData{u}.Task-1) - ReAll.timeFh(u)
    if taup > taup1
        indexVal{u} = indexVal{u} - (1/humanData{u}.WaitingTime(humanData{u}.Task-1))*(abs(taup - taup1)/taup)
    elseif taup < taup1
        indexVal{u} = indexVal{u} + (1/humanData{u}.WaitingTime(humanData{u}.Task-1))*(abs(taup - taup1)/taup)
    elseif taup == taup1
        indexVal{u} = indexVal{u}
    end
end

if humanData{u}.WaitingTime(humanData{u}.Task-1) > 0 % human asks for a shorter waiting time
    taup = ReAll.timeSh(u+num_agents*humanData{u}.Task) - ReAll.timeFh(u+num_agents)
    taup1 = ReAll.timeSh(u+num_agents*humanData{u}.Task-1) - ReAll.timeFh(u)
    if taup > taup1
        indexVal{u} = indexVal{u} - (1/humanData{u}.WaitingTime(humanData{u}.Task-1))*(abs(taup - taup1)/taup)
    elseif taup < taup1
        indexVal{u} = indexVal{u} + (1/humanData{u}.WaitingTime(humanData{u}.Task-1))*(abs(taup - taup1)/taup)
    elseif taup == taup1
        indexVal{u} = indexVal{u}
    end
end

%% Without ReAll
if humanData{u}.WaitingTime(humanData{u}.Task-1) < 0
    taupW = FirstReAll.timeSh(u+num_agents*humanData{u}.Task) - FirstReAll.timeFh(u+num_agents)
    taupW1 = FirstReAll.timeSh(u+num_agents*humanData{u}.Task-1) - FirstReAll.timeFh(u)
    if taupW > taupW1
        indexValW{u} = indexValW{u} - (1/humanData{u}.WaitingTime(humanData{u}.Task-1))*(abs(taupW - taupW1)/taupW)
    elseif taupW < taupW1
        indexValW{u} = indexValW{u} + (1/humanData{u}.WaitingTime(humanData{u}.Task-1))*(abs(taupW - taupW1)/taupW)
    elseif taupW == taupW1
        indexValW{u} = indexVal{u}
    end
end

if humanData{u}.WaitingTime(humanData{u}.Task-1) > 0
    taupW = FirstReAll.timeSh(u+num_agents*humanData{u}.Task) - FirstReAll.timeFh(u+num_agents)
    taupW1 = FirstReAll.timeSh(u+num_agents*humanData{u}.Task-1) - FirstReAll.timeFh(u)
    if taupW > taupW1
        indexValW{u} = indexValW{u} - (1/humanData{u}.WaitingTime(humanData{u}.Task-1))*(abs(taupW - taupW1)/taupW)
    elseif taupW < taupW1
        indexValW{u} = indexValW{u} + (1/humanData{u}.WaitingTime(humanData{u}.Task-1))*(abs(taupW - taupW1)/taupW)
    elseif taupW == taupW1
        indexValW{u} = indexVal{u}
    end
end

indexVal
indexValW