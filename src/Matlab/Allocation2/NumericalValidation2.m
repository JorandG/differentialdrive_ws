function NumericalValidation(ReAll, humanData, u)
    global compteur num_agents indexVal indexValW FirstReAll storedIndexVal storedIndexValW numstored ReAllSave num_humans
    
    if isempty(storedIndexVal)
        storedIndexVal = {}; % Initialize storage for indexVal
    end
    
    if isempty(storedIndexValW)
        storedIndexValW = {}; % Initialize storage for indexValW
    end

    waitingtime = [];

    for hum=1:num_humans
        waitingtime(end+1) = FirstReAll.timeSh(hum+num_agents*(2)) - FirstReAll.timeFh(hum+num_agents*(1))
    end
    
    averagewaiting = mean(waitingtime)

    if humanData{u}.WaitingTime(1) < 0 % human asks for a longer waiting time
        taup = ReAllSave.timeSh(u+num_agents*(1)) - ReAllSave.timeFh(u+num_agents*(0));
        taup1 = ReAllSave.timeSh(u+num_agents*(2)) - ReAllSave.timeFh(u+num_agents*(1));
        if taup > taup1
            indexVal{u} = indexVal{u} - abs((1/humanData{u}.WaitingTime(1))*(abs(taup - taup1)/averagewaiting));
        elseif taup < taup1
            indexVal{u} = indexVal{u} + abs((1/humanData{u}.WaitingTime(1))*(abs(taup - taup1)/averagewaiting));
        end
    end

    if humanData{u}.WaitingTime(1) > 0 % human asks for a shorter waiting time
        taup = ReAllSave.timeSh(u+num_agents*(1)) - ReAllSave.timeFh(u+num_agents*(0));
        taup1 = ReAllSave.timeSh(u+num_agents*(2)) - ReAllSave.timeFh(u+num_agents*(1));
        if taup > taup1
            indexVal{u} = indexVal{u} + abs((1/humanData{u}.WaitingTime(1))*(abs(taup - taup1)/averagewaiting));
        elseif taup < taup1
            indexVal{u} = indexVal{u} - abs((1/humanData{u}.WaitingTime(1))*(abs(taup - taup1)/averagewaiting));
        end
    end

    % Store the current value of indexVal{u}
    storedIndexVal{u}{1} = indexVal{u};
    
    %% Without ReAll
    if humanData{u}.WaitingTime(1) < 0
        taupW = FirstReAll.timeSh(u+num_agents*(1)) - FirstReAll.timeFh(u+num_agents*(0));
        taupW1 = FirstReAll.timeSh(u+num_agents*(2)) - FirstReAll.timeFh(u+num_agents*(1));
        if taupW > taupW1
            indexValW{u} = indexValW{u} - abs((1/humanData{u}.WaitingTime(1))*(abs(taupW - taupW1)/averagewaiting));
        elseif taupW < taupW1
            indexValW{u} = indexValW{u} + abs((1/humanData{u}.WaitingTime(1))*(abs(taupW - taupW1)/averagewaiting));
        end
    end

    if humanData{u}.WaitingTime(1) > 0
        taupW = FirstReAll.timeSh(u+num_agents*1) - FirstReAll.timeFh(u+num_agents*(0));
        taupW1 = FirstReAll.timeSh(u+num_agents*(2)) - FirstReAll.timeFh(u+num_agents*(1));
        if taupW > taupW1
            indexValW{u} = indexValW{u} + abs((1/humanData{u}.WaitingTime(1))*(abs(taupW - taupW1)/averagewaiting));
        elseif taupW < taupW1
            indexValW{u} = indexValW{u} - abs((1/humanData{u}.WaitingTime(1))*(abs(taupW - taupW1)/averagewaiting));
        end
    end

    % Store the current value of indexValW{u}
    storedIndexValW{u}{1} = indexValW{u};
    
    indexVal
    indexValW
    compteur = compteur + 1;
end
