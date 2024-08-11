global timingData humanData pub ReAllSave MILPData
global flags feedback

% Ensure feedback is initialized
if isempty(feedback) && ~flags(28)
    feedback = arrayfun(@(~) randsample([-1, -0.5, 0.5, 1], 1), 1:2); % Only 2 feedback values for 2 humans
    flags(28) = true;
end

% Start Filling 1
if any(timingData >= ReAllSave.timeSh(1)) && ~flags(1)
    humanData{1}.StartFilling(1) = ReAllSave.timeSh(1);
    flags(1) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= ReAllSave.timeSh(2)) && ~flags(2)
    humanData{2}.StartFilling(1) = ReAllSave.timeSh(2);
    flags(2) = true;
    send(pub{2}, humanData{2});
end

% Finish Filling & Start Serving 1
if any(timingData >= ReAllSave.timeFh(1)) && ~flags(4)
    humanData{1}.FinishFilling(1) = ReAllSave.timeFh(1);
    humanData{1}.ConfirmFilling(1) = 1;
    humanData{1}.StartServing(1) = ReAllSave.timeFh(1);
    flags(4) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= ReAllSave.timeFh(2)) && ~flags(5)
    humanData{2}.FinishFilling(1) = ReAllSave.timeFh(2);
    humanData{2}.ConfirmFilling(1) = 1;
    humanData{2}.StartServing(1) = ReAllSave.timeFh(2);
    flags(5) = true;
    send(pub{2}, humanData{2});
end

% Finish Serving 1
if any(timingData >= ReAllSave.timeF(19)) && ~flags(7)
    humanData{1}.FinishServing(1) = ReAllSave.timeF(28);
    humanData{1}.ConfirmServing(1) = 1;
    humanData{1}.RobotVelocityProximity(1) = -1; % Very Fast
    humanData{1}.WaitingTime(1) = feedback(1);
    humanData{1}.RobotWaitingDistance(1) = 0.5;
    flags(7) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= ReAllSave.timeF(20)) && ~flags(8)
    humanData{2}.FinishServing(1) = ReAllSave.timeF(29);
    humanData{2}.ConfirmServing(1) = 1;
    humanData{2}.RobotVelocityProximity(1) = 1; % Very Slow
    humanData{2}.WaitingTime(1) = feedback(2);
    humanData{2}.RobotWaitingDistance(2) = 1;
    flags(8) = true;
    send(pub{2}, humanData{2});
end

% Start Filling 2
if any(timingData >= ReAllSave.timeSh(3)) && ~flags(10)
    humanData{1}.StartFilling(2) = ReAllSave.timeSh(3);
    flags(10) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= ReAllSave.timeSh(4)) && ~flags(11)
    humanData{2}.StartFilling(2) = ReAllSave.timeSh(4);
    flags(11) = true;
    send(pub{2}, humanData{2});
end

% Finish Filling & Start Serving 2
if any(timingData >= ReAllSave.timeFh(3)) && ~flags(13)
    humanData{1}.FinishFilling(2) = ReAllSave.timeFh(3);
    humanData{1}.ConfirmFilling(2) = 1;
    humanData{1}.StartServing(2) = ReAllSave.timeFh(3);
    flags(13) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= ReAllSave.timeFh(4)) && ~flags(14)
    humanData{2}.FinishFilling(2) = ReAllSave.timeFh(4);
    humanData{2}.ConfirmFilling(2) = 1;
    humanData{2}.StartServing(2) = ReAllSave.timeFh(4);
    flags(14) = true;
    send(pub{2}, humanData{2});
end

% Finish Serving 2
if any(timingData >= ReAllSave.timeF(21)) && ~flags(16)
    humanData{1}.FinishServing(2) = ReAllSave.timeF(21);
    humanData{1}.ConfirmServing(2) = 1;
    humanData{1}.RobotVelocityProximity(2) = -0.5; % Fast
    humanData{1}.WaitingTime(2) = feedback(1);
    humanData{1}.RobotWaitingDistance(2) = 1;
    flags(16) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= ReAllSave.timeF(22)) && ~flags(17)
    humanData{2}.FinishServing(2) = ReAllSave.timeF(22);
    humanData{2}.ConfirmServing(2) = 1;
    humanData{2}.RobotVelocityProximity(2) = 0.5; % Slow
    humanData{2}.WaitingTime(2) = feedback(2);
    humanData{2}.RobotWaitingDistance(2) = 1;
    flags(17) = true;
    send(pub{2}, humanData{2});
end

% Start Filling 3
if any(timingData >= ReAllSave.timeSh(5)) && ~flags(19)
    humanData{1}.StartFilling(3) = ReAllSave.timeSh(5);
    flags(19) = true;
    send(pub{1}, humanData{1}); 
elseif any(timingData >= ReAllSave.timeSh(6)) && ~flags(20)
    humanData{2}.StartFilling(3) = ReAllSave.timeSh(6);
    flags(20) = true;
    send(pub{2}, humanData{2});
end

% Finish Filling & Start Serving 3
if any(timingData >= ReAllSave.timeFh(5)) && ~flags(22)
    humanData{1}.FinishFilling(3) = ReAllSave.timeFh(5);
    humanData{1}.ConfirmFilling(3) = 1;
    humanData{1}.StartServing(3) = ReAllSave.timeFh(5);
    flags(22) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= ReAllSave.timeFh(6)) && ~flags(23)
    humanData{2}.FinishFilling(3) = ReAllSave.timeFh(6);
    humanData{2}.ConfirmFilling(3) = 1;
    humanData{2}.StartServing(3) = ReAllSave.timeFh(6);
    flags(23) = true;
    send(pub{2}, humanData{2});
end

% Finish Serving 3
if any(timingData >= ReAllSave.timeF(23)) && ~flags(25)
    humanData{1}.FinishServing(3) = ReAllSave.timeF(23);
    humanData{1}.ConfirmServing(3) = 1;
    humanData{1}.RobotVelocityProximity(3) = 0; % Ok
    humanData{1}.WaitingTime(3) = feedback(1);
    humanData{1}.RobotWaitingDistance(3) = 0.5;
    flags(25) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= ReAllSave.timeF(24)) && ~flags(26)
    humanData{2}.FinishServing(3) = ReAllSave.timeF(24);
    humanData{2}.ConfirmServing(3) = 1;  
    humanData{2}.RobotVelocityProximity(3) = 0; % Ok
    humanData{2}.WaitingTime(3) = feedback(2);
    humanData{2}.RobotWaitingDistance(3) = 1.5;
    flags(26) = true;
    send(pub{2}, humanData{2});
end
