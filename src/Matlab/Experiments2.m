global timingData humanData pub
global flags

%Start Filling 1
if any(timingData >= 8) && ~flags(1)
    humanData{1}.StartFilling(1) = 8;
    flags(1) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= 10) && ~flags(2)
    humanData{2}.StartFilling(1) = 10;
    flags(2) = true;
    send(pub{2}, humanData{2});
end

%Finish Filling & Start Serving 1
if any(timingData >= 65) && ~flags(3)
    humanData{1}.FinishFilling(1) = 65;
    humanData{1}.ConfirmFilling(1) = 1;
    humanData{1}.StartServing(1) = 65;
    flags(3) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= 130) && ~flags(4)
    humanData{2}.FinishFilling(1) = 130;
    humanData{2}.ConfirmFilling(1) = 1;
    humanData{2}.StartServing(1) = 130;
    flags(4) = true;
    send(pub{2}, humanData{2});
end

%Finish Serving 1
if any(timingData >= 71) && ~flags(5)
    humanData{1}.FinishServing(1) = 71;
    humanData{1}.ConfirmServing(1) = 1;    
    humanData{1}.RobotVelocityProximity(1) = -1; %Very Fast
    humanData{1}.WaitingTime(1) = 1; %Waiting time too high
    humanData{1}.RobotWaitingDistance(1) = 0.5;
    flags(5) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= 162) && ~flags(6)
    humanData{2}.FinishServing(1) = 162;
    humanData{2}.ConfirmServing(1) = 1;    
    humanData{2}.RobotVelocityProximity(1) = 1; %Very Slow
    humanData{2}.WaitingTime(1) = -1; %Waiting time too loww
    humanData{2}.RobotWaitingDistance(2) = 1;
    flags(6) = true;
    send(pub{2}, humanData{2});        
end

%Start Filling 2
if any(timingData >= 153) && ~flags(7)
    humanData{1}.StartFilling(2) = 153;
    flags(7) = true;
    send(pub{1}, humanData{1});
    % 
    % ProximityTaskFeedback = [ProximityTaskFeedback, humanData{1}.RobotVelocityProximity(1)'];
    % ProximityTaskFeedback = [ProximityTaskFeedback, humanData{2}.RobotVelocityProximity(1)'];
    % ProximityTaskWeights = [ProximityTaskWeights, humanData{1}.RobotVelocityProximityWeight(1)'];
    % ProximityTaskWeights = [ProximityTaskWeights, humanData{2}.RobotVelocityProximityWeight(1)'];
    % ProximityTaskDurations = [ProximityTaskDurations, (ReAllSave.timeF(idx_approaching_tasks) - ReAllSave.timeS(idx_approaching_tasks))];  
elseif any(timingData >= 170) && ~flags(8)
    humanData{2}.StartFilling(2) = 170;
    flags(8) = true;
    send(pub{2}, humanData{2});
end

%Finish Filling & Start Serving 2
if any(timingData >= 250) && ~flags(9)
    humanData{1}.FinishFilling(2) = 250;
    humanData{1}.ConfirmFilling(2) = 1;
    humanData{1}.StartServing(2) = 250;
    flags(9) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= 325) && ~flags(10)
    humanData{2}.FinishFilling(2) = 325;
    humanData{2}.ConfirmFilling(2) = 1;
    humanData{2}.StartServing(2) = 325;
    flags(10) = true;
    send(pub{2}, humanData{2});
end

%Finish Serving 2
if any(timingData >= 257) && ~flags(11)
    humanData{1}.FinishServing(2) = 257;
    humanData{1}.ConfirmServing(2) = 1;
    humanData{1}.RobotVelocityProximity(2) = -0.5; %Fast
    humanData{1}.WaitingTime(2) = 0.5; %Waiting time high
    humanData{1}.RobotWaitingDistance(2) = 1;
    flags(11) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= 335) && ~flags(12)
    humanData{2}.FinishServing(2) = 335;
    humanData{2}.ConfirmServing(2) = 1;  
    humanData{2}.RobotVelocityProximity(2) = 0.5; %Slow
    humanData{2}.WaitingTime(2) = -0.5; %Waiting time low
    humanData{2}.RobotWaitingDistance(2) = 1;
    flags(12) = true;
    send(pub{2}, humanData{2});
end

%Start Filling 3
if any(timingData >= 326) && ~flags(13)
    humanData{1}.StartFilling(3) = 326;
    flags(13) = true;
    send(pub{1}, humanData{1});
    % 
    % ProximityTaskFeedback = [ProximityTaskFeedback, humanData{1}.RobotVelocityProximity(2)'];
    % ProximityTaskFeedback = [ProximityTaskFeedback, humanData{2}.RobotVelocityProximity(2)'];
    % ProximityTaskWeights = [ProximityTaskWeights, humanData{1}.RobotVelocityProximityWeight(2)'];
    % ProximityTaskWeights = [ProximityTaskWeights, humanData{2}.RobotVelocityProximityWeight(2)'];
    % ProximityTaskDurations = [ProximityTaskDurations, (ReAllSave.timeF(idx_approaching_tasks) - ReAllSave.timeS(idx_approaching_tasks))];  
elseif any(timingData >= 341) && ~flags(14)
    humanData{2}.StartFilling(3) = 341;
    flags(14) = true;
    send(pub{2}, humanData{2});
end

%Finish Filling & Start Serving 3
if any(timingData >= 435) && ~flags(15)
    humanData{1}.FinishFilling(3) = 435;
    humanData{1}.ConfirmFilling(3) = 1;
    humanData{1}.StartServing(3) = 435;
    flags(15) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= 498) && ~flags(16)
    humanData{2}.FinishFilling(3) = 498;
    humanData{2}.ConfirmFilling(3) = 1;
    humanData{2}.StartServing(3) = 498;
    flags(16) = true;
    send(pub{2}, humanData{2});
end

%Finish Serving 3
if any(timingData >= 442) && ~flags(17)
    humanData{1}.FinishServing(3) = 442;
    humanData{1}.ConfirmServing(3) = 1;
    humanData{1}.RobotVelocityProximity(3) = 0; %Ok
    humanData{1}.WaitingTime(3) = 0; %Ok
    humanData{1}.RobotWaitingDistance(3) = 0.5;
    flags(17) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= 508) && ~flags(18)
    humanData{2}.FinishServing(3) = 508;
    humanData{2}.ConfirmServing(3) = 1;  
    humanData{2}.RobotVelocityProximity(3) = 0; %Ok
    humanData{2}.WaitingTime(3) = 0; %Ok
    humanData{2}.RobotWaitingDistance(3) = 1.5;
    flags(18) = true;
    send(pub{2}, humanData{2});
end

if any(timingData >= 540) && ~flags(19)
    flags(19) = true;
    ProximityTaskFeedback = [ProximityTaskFeedback, humanData{1}.RobotVelocityProximity(3)'];
    ProximityTaskFeedback = [ProximityTaskFeedback, humanData{2}.RobotVelocityProximity(3)'];
    ProximityTaskWeights = [ProximityTaskWeights, humanData{1}.RobotVelocityProximityWeight(3)'];
    ProximityTaskWeights = [ProximityTaskWeights, humanData{2}.RobotVelocityProximityWeight(3)'];
    ProximityTaskDurations = [ProximityTaskDurations, (ReAllSave.timeF(idx_approaching_tasks) - ReAllSave.timeS(idx_approaching_tasks))];
end
