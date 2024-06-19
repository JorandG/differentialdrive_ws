global timingData humanData pub

%Start Filling 1
if any(timingData >= 8) && any(timingData <= 8.5)
    humanData{1}.StartFilling(1) = timingData;
    send(pub{1}, humanData{1});
elseif any(timingData >= 10) && any(timingData <= 10.5)
    humanData{2}.StartFilling(1) = timingData;
    send(pub{2}, humanData{2});
end

%Finish Filling & Start Serving 1
if any(timingData >= 115) && any(timingData <= 115.5)
    humanData{1}.ConfirmFilling(1) = 1;
    humanData{1}.StartServing(1) = timingData;
    send(pub{1}, humanData{1});
elseif any(timingData >= 170) && any(timingData <= 170.5)
    humanData{2}.ConfirmFilling(1) = 1;
    humanData{2}.StartServing(1) = timingData;
    send(pub{2}, humanData{2});
end

%Finish Serving 1
if any(timingData >= 121) && any(timingData <= 121.5)
    humanData{1}.ConfirmServing(1) = 1;    
    humanData{1}.RobotVelocityProximity(1) = 1; %Very Fast
    send(pub{1}, humanData{1});
elseif any(timingData >= 177) && any(timingData <= 177.5)
    humanData{2}.ConfirmServing(1) = 1;    
    humanData{2}.RobotVelocityProximity(1) = -1; %Very Slow
    send(pub{2}, humanData{2});        
end

%Start Filling 2
if any(timingData >= 203) && any(timingData <= 203.1)
    humanData{1}.StartFilling(2) = timingData;
    send(pub{1}, humanData{1});

    ProximityTaskFeedback = [ProximityTaskFeedback, humanData{1}.RobotVelocityProximity(1)']
    ProximityTaskFeedback = [ProximityTaskFeedback, humanData{2}.RobotVelocityProximity(1)']
    ProximityTaskWeights = [ProximityTaskWeights, humanData{1}.RobotVelocityProximityWeight(1)']
    ProximityTaskWeights = [ProximityTaskWeights, humanData{2}.RobotVelocityProximityWeight(1)']
    ProximityTaskDurations = [ProximityTaskDurations, (ReAllSave.timeF(idx_approaching_tasks) - ReAllSave.timeS(idx_approaching_tasks))]  
elseif any(timingData >= 210) && any(timingData <= 210.1)
    humanData{2}.StartFilling(2) = timingData;
    send(pub{2}, humanData{2});
end

%Finish Filling & Start Serving 2
if any(timingData >= 300) && any(timingData <= 300.1)
    humanData{1}.ConfirmFilling(2) = 1;
    humanData{1}.StartServing(2) = timingData;
    send(pub{1}, humanData{1});
elseif any(timingData >= 365) && any(timingData <= 365.1)
    humanData{2}.ConfirmFilling(2) = 1;
    humanData{2}.StartServing(2) = timingData;
    send(pub{2}, humanData{2});
end

%Finish Serving 2
if any(timingData >= 307) && any(timingData <= 307.1)
    humanData{1}.ConfirmServing(2) = 1;
    humanData{1}.RobotVelocityProximity(2) = 0.5; %Fast
    send(pub{1}, humanData{1});
elseif any(timingData >= 375) && any(timingData <= 375.1)
    humanData{2}.ConfirmServing(2) = 1;  
    humanData{2}.RobotVelocityProximity(2) = -0.5; %Slow
    send(pub{2}, humanData{2});
end






%Start Filling 3
if any(timingData >= 374) && any(timingData <= 374.1)
    humanData{1}.StartFilling(3) = timingData;
    send(pub{1}, humanData{1});

    ProximityTaskFeedback = [ProximityTaskFeedback, humanData{1}.RobotVelocityProximity(2)']
    ProximityTaskFeedback = [ProximityTaskFeedback, humanData{2}.RobotVelocityProximity(2)']
    ProximityTaskWeights = [ProximityTaskWeights, humanData{1}.RobotVelocityProximityWeight(2)']
    ProximityTaskWeights = [ProximityTaskWeights, humanData{2}.RobotVelocityProximityWeight(2)']
    ProximityTaskDurations = [ProximityTaskDurations, (ReAllSave.timeF(idx_approaching_tasks) - ReAllSave.timeS(idx_approaching_tasks))]  
elseif any(timingData >= 381) && any(timingData <= 381.1)
    humanData{2}.StartFilling(3) = timingData;
    send(pub{2}, humanData{2});
end

%Finish Filling & Start Serving 3
if any(timingData >= 485) && any(timingData <= 485.5)
    humanData{1}.ConfirmFilling(3) = 1;
    humanData{1}.StartServing(3) = timingData;
    send(pub{1}, humanData{1});
elseif any(timingData >= 538) && any(timingData <= 538.5)
    humanData{2}.ConfirmFilling(3) = 1;
    humanData{2}.StartServing(3) = timingData;
    send(pub{2}, humanData{2});
end

%Finish Serving 3
if any(timingData >= 492) && any(timingData <= 492.5)
    humanData{1}.ConfirmServing(3) = 1;
    humanData{1}.RobotVelocityProximity(3) = 0; %Ok
    send(pub{1}, humanData{1});
elseif any(timingData >= 548) && any(timingData <= 548.5)
    humanData{2}.ConfirmServing(3) = 1;  
    humanData{2}.RobotVelocityProximity(3) = 0; %Ok
    send(pub{2}, humanData{2});
end

if any(timingData >= 590) && any(timingData <= 590.1)
    ProximityTaskFeedback = [ProximityTaskFeedback, humanData{1}.RobotVelocityProximity(3)']
    ProximityTaskFeedback = [ProximityTaskFeedback, humanData{2}.RobotVelocityProximity(3)']
    ProximityTaskWeights = [ProximityTaskWeights, humanData{1}.RobotVelocityProximityWeight(3)']
    ProximityTaskWeights = [ProximityTaskWeights, humanData{2}.RobotVelocityProximityWeight(3)']
    ProximityTaskDurations = [ProximityTaskDurations, (ReAllSave.timeF(idx_approaching_tasks) - ReAllSave.timeS(idx_approaching_tasks))] 
end
