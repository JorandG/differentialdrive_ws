global timingData humanData pub ReAllSave MILPData
global flags feedback

%  Start Filling 1
if any(timingData >= ReAllSave.timeSh(1)) && ~flags(1)
    humanData{1}.StartFilling(1) = ReAllSave.timeSh(1);
    flags(1) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= ReAllSave.timeSh(2)) && ~flags(2)
    humanData{2}.StartFilling(1) = ReAllSave.timeSh(2);
    flags(2) = true;
    send(pub{2}, humanData{2});
elseif any(timingData >= ReAllSave.timeSh(3)) && ~flags(3)
    humanData{3}.StartFilling(1) = ReAllSave.timeSh(3);
    flags(3) = true;
    send(pub{3}, humanData{3});
elseif any(timingData >= ReAllSave.timeSh(4)) && ~flags(4)
    humanData{4}.StartFilling(1) = ReAllSave.timeSh(4);
    flags(4) = true;
    send(pub{4}, humanData{4});
end

% Finish Filling & Start Serving 1
if any(timingData >= ReAllSave.timeFh(1)) && ~flags(5)
    humanData{1}.FinishFilling(1) = ReAllSave.timeFh(1);
    humanData{1}.ConfirmFilling(1) = 1;
    humanData{1}.StartServing(1) = ReAllSave.timeFh(1);
    flags(5) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= ReAllSave.timeFh(2)) && ~flags(6)
    humanData{2}.FinishFilling(1) = ReAllSave.timeFh(2);
    humanData{2}.ConfirmFilling(1) = 1;
    humanData{2}.StartServing(1) = ReAllSave.timeFh(2);
    flags(6) = true;
    send(pub{2}, humanData{2});
elseif any(timingData >= ReAllSave.timeFh(3)) && ~flags(7)
    humanData{3}.FinishFilling(1) = ReAllSave.timeFh(3);
    humanData{3}.ConfirmFilling(1) = 1;
    humanData{3}.StartServing(1) = ReAllSave.timeFh(3);
    flags(7) = true;
    send(pub{3}, humanData{3});
elseif any(timingData >= ReAllSave.timeFh(4)) && ~flags(8)
    humanData{4}.FinishFilling(1) = ReAllSave.timeFh(4);
    humanData{4}.ConfirmFilling(1) = 1;
    humanData{4}.StartServing(1) = ReAllSave.timeFh(4);
    flags(8) = true;
    send(pub{4}, humanData{4});
end

% Finish Serving 1
if any(timingData >= ReAllSave.timeF(37)) && ~flags(9)
    humanData{1}.FinishServing(1) = ReAllSave.timeF(37);
    humanData{1}.ConfirmServing(1) = 1;
    humanData{1}.RobotVelocityProximity(1) = -1; % Very Fast
    humanData{1}.RobotWaitingDistance(1) = 0.5;
    flags(9) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= ReAllSave.timeF(38)) && ~flags(10)
    humanData{2}.FinishServing(1) = ReAllSave.timeF(38);
    humanData{2}.ConfirmServing(1) = 1;
    humanData{2}.RobotVelocityProximity(1) = 1; % Very Slow
    humanData{2}.RobotWaitingDistance(2) = 1;
    flags(10) = true;
    send(pub{2}, humanData{2});
elseif any(timingData >= ReAllSave.timeF(39)) && ~flags(11)
    humanData{3}.FinishServing(1) = ReAllSave.timeF(39);
    humanData{3}.ConfirmServing(1) = 1;
    humanData{3}.RobotVelocityProximity(1) = 0.5; % Slow
    humanData{3}.RobotWaitingDistance(3) = 1.5;
    flags(11) = true;
    send(pub{3}, humanData{3});
elseif any(timingData >= ReAllSave.timeF(40)) && ~flags(12)
    humanData{4}.FinishServing(1) = ReAllSave.timeF(40);
    humanData{4}.ConfirmServing(1) = 1;
    humanData{4}.RobotVelocityProximity(1) = 0; % Moderate
    humanData{4}.RobotWaitingDistance(4) = 1.5;
    flags(12) = true;
    send(pub{4}, humanData{4});
end

% Start Filling 2
if any(timingData >= ReAllSave.timeSh(5)) && ~flags(13)
    humanData{1}.StartFilling(2) = ReAllSave.timeSh(5);
    flags(13) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= ReAllSave.timeSh(6)) && ~flags(14)
    humanData{2}.StartFilling(2) = ReAllSave.timeSh(6);
    flags(14) = true;
    send(pub{2}, humanData{2});
elseif any(timingData >= ReAllSave.timeSh(7)) && ~flags(15)
    humanData{3}.StartFilling(2) = ReAllSave.timeSh(7);
    flags(15) = true;
    send(pub{3}, humanData{3});
elseif any(timingData >= ReAllSave.timeSh(8)) && ~flags(16)
    humanData{4}.StartFilling(2) = ReAllSave.timeSh(8);
    flags(16) = true;
    send(pub{4}, humanData{4});
end

% Finish Filling & Start Serving 2
if any(timingData >= ReAllSave.timeFh(5)) && ~flags(17)
    humanData{1}.FinishFilling(2) = ReAllSave.timeFh(5);
    humanData{1}.ConfirmFilling(2) = 1;
    humanData{1}.StartServing(2) = ReAllSave.timeFh(5);
    flags(17) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= ReAllSave.timeFh(6)) && ~flags(18)
    humanData{2}.FinishFilling(2) = ReAllSave.timeFh(6);
    humanData{2}.ConfirmFilling(2) = 1;
    humanData{2}.StartServing(2) = ReAllSave.timeFh(6);
    flags(18) = true;
    send(pub{2}, humanData{2});
elseif any(timingData >= ReAllSave.timeFh(7)) && ~flags(19)
    humanData{3}.FinishFilling(2) = ReAllSave.timeFh(7);
    humanData{3}.ConfirmFilling(2) = 1;
    humanData{3}.StartServing(2) = ReAllSave.timeFh(7);
    flags(19) = true;
    send(pub{3}, humanData{3});
elseif any(timingData >= ReAllSave.timeFh(8)) && ~flags(20)
    humanData{4}.FinishFilling(2) = ReAllSave.timeFh(8);
    humanData{4}.ConfirmFilling(2) = 1;
    humanData{4}.StartServing(2) = ReAllSave.timeFh(8);
    flags(20) = true;
    send(pub{4}, humanData{4});
end

% Finish Serving 2
if any(timingData >= ReAllSave.timeF(41)) && ~flags(21)
    humanData{1}.FinishServing(2) = ReAllSave.timeF(41);
    humanData{1}.ConfirmServing(2) = 1;
    humanData{1}.RobotVelocityProximity(2) = -0.5; % Fast
    humanData{1}.RobotWaitingDistance(2) = 1;
    flags(21) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= ReAllSave.timeF(42)) && ~flags(22)
    humanData{2}.FinishServing(2) = ReAllSave.timeF(42);
    humanData{2}.ConfirmServing(2) = 1;
    humanData{2}.RobotVelocityProximity(2) = 0.5; % Slow
    humanData{2}.RobotWaitingDistance(2) = 1;
    flags(22) = true;
    send(pub{2}, humanData{2});
elseif any(timingData >= ReAllSave.timeF(43)) && ~flags(23)
    humanData{3}.FinishServing(2) = ReAllSave.timeF(43);
    humanData{3}.ConfirmServing(2) = 1;
    humanData{3}.RobotVelocityProximity(2) = 0; % Moderate
    humanData{3}.RobotWaitingDistance(3) = 1.5;
    flags(23) = true;
    send(pub{3}, humanData{3});
elseif any(timingData >= ReAllSave.timeF(44)) && ~flags(24)
    humanData{4}.FinishServing(2) = ReAllSave.timeF(44);
    humanData{4}.ConfirmServing(2) = 1;
    humanData{4}.RobotVelocityProximity(2) = 0.5; % Slow
    humanData{4}.RobotWaitingDistance(4) = 1.5;
    flags(24) = true;
    send(pub{4}, humanData{4});
end

% Start Filling 3
if any(timingData >= ReAllSave.timeSh(9)) && ~flags(25)
    humanData{1}.StartFilling(3) = ReAllSave.timeSh(9);
    flags(25) = true;
    send(pub{1}, humanData{1}); 
elseif any(timingData >= ReAllSave.timeSh(10)) && ~flags(26)
    humanData{2}.StartFilling(3) = ReAllSave.timeSh(10);
    flags(26) = true;
    send(pub{2}, humanData{2});
elseif any(timingData >= ReAllSave.timeSh(11)) && ~flags(27)
    humanData{3}.StartFilling(3) = ReAllSave.timeSh(11);
    flags(27) = true;
    send(pub{3}, humanData{3});
elseif any(timingData >= ReAllSave.timeSh(12)) && ~flags(28)
    humanData{4}.StartFilling(3) = ReAllSave.timeSh(12);
    flags(28) = true;
    send(pub{4}, humanData{4});
end

% Finish Filling & Start Serving 3
if any(timingData >= ReAllSave.timeFh(9)) && ~flags(29)
    humanData{1}.FinishFilling(3) = ReAllSave.timeFh(9);
    humanData{1}.ConfirmFilling(3) = 1;
    humanData{1}.StartServing(3) = ReAllSave.timeFh(9);
    flags(29) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= ReAllSave.timeFh(10)) && ~flags(30)
    humanData{2}.FinishFilling(3) = ReAllSave.timeFh(10);
    humanData{2}.ConfirmFilling(3) = 1;
    humanData{2}.StartServing(3) = ReAllSave.timeFh(10);
    flags(30) = true;
    send(pub{2}, humanData{2});
elseif any(timingData >= ReAllSave.timeFh(11)) && ~flags(31)
    humanData{3}.FinishFilling(3) = ReAllSave.timeFh(11);
    humanData{3}.ConfirmFilling(3) = 1;
    humanData{3}.StartServing(3) = ReAllSave.timeFh(11);
    flags(31) = true;
    send(pub{3}, humanData{3});
elseif any(timingData >= ReAllSave.timeFh(12)) && ~flags(32)
    humanData{4}.FinishFilling(3) = ReAllSave.timeFh(12);
    humanData{4}.ConfirmFilling(3) = 1;
    humanData{4}.StartServing(3) = ReAllSave.timeFh(12);
    flags(32) = true;
    send(pub{4}, humanData{4});
end

% Finish Serving 3
if any(timingData >= ReAllSave.timeF(45)) && ~flags(33)
    humanData{1}.FinishServing(3) = ReAllSave.timeF(45);
    humanData{1}.ConfirmServing(3) = 1;
    humanData{1}.RobotVelocityProximity(3) = 0; % Ok
    humanData{1}.RobotWaitingDistance(3) = 0.5;
    flags(33) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= ReAllSave.timeF(46)) && ~flags(34)
    humanData{2}.FinishServing(3) = ReAllSave.timeF(46);
    humanData{2}.ConfirmServing(3) = 1;  
    humanData{2}.RobotVelocityProximity(3) = 0; % Ok
    humanData{2}.RobotWaitingDistance(3) = 1.5;
    flags(34) = true;
    send(pub{2}, humanData{2});
elseif any(timingData >= ReAllSave.timeF(47)) && ~flags(35)
    humanData{3}.FinishServing(3) = ReAllSave.timeF(47);
    humanData{3}.ConfirmServing(3) = 1;  
    humanData{3}.RobotVelocityProximity(3) = 0; % Ok
    humanData{3}.RobotWaitingDistance(3) = 1.5;
    flags(35) = true;
    send(pub{3}, humanData{3});
elseif any(timingData >= ReAllSave.timeF(48)) && ~flags(36)
    humanData{4}.FinishServing(3) = ReAllSave.timeF(48);
    humanData{4}.ConfirmServing(3) = 1;  
    humanData{4}.RobotVelocityProximity(3) = 0; % Ok
    humanData{4}.RobotWaitingDistance(3) = 1.5;
    flags(36) = true;
    send(pub{4}, humanData{4});
end
