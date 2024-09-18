global timingData humanData pub ReAllSave MILPData finfill finserv
global flags feedback num_filling_boxes ReAll slowlier

% Start Filling 1
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
if any(timingData >= ReAllSave.timeFh(1)-20) && ~flags(5)
    finfill = true;
    humanData{1}.FinishFilling(1) = ReAllSave.timeFh(1)-20;
    humanData{1}.ConfirmFilling(1) = 1;
    humanData{1}.StartServing(1) = ReAllSave.timeS(37);
    flags(5) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= ReAllSave.timeFh(2)+60) && ~flags(6)
    finfill = true;
    humanData{2}.FinishFilling(1) = ReAllSave.timeFh(2)+60;
    %humanData{2}.ConfirmFilling(1) = 1;
    humanData{2}.StartServing(1) = ReAllSave.timeS(38);
    flags(6) = true;
    send(pub{2}, humanData{2});
elseif any(timingData >= ReAllSave.timeFh(2)+61) && ~flags(38)
    humanData{2}.ConfirmFilling(1) = 1;
    flags(38) = true;
    send(pub{2}, humanData{2});
elseif any(timingData >= ReAllSave.timeFh(3)-10) && ~flags(7)
    finfill = true;
    humanData{3}.FinishFilling(1) = ReAllSave.timeFh(3)-10;
    humanData{3}.ConfirmFilling(1) = 1;
    humanData{3}.StartServing(1) = ReAllSave.timeS(39);
    flags(7) = true;
    send(pub{3}, humanData{3});
elseif any(timingData >= ReAllSave.timeFh(4)) && ~flags(8)
    finfill = true;
    humanData{4}.FinishFilling(1) = ReAllSave.timeFh(4);
    humanData{4}.ConfirmFilling(1) = 1;
    humanData{4}.StartServing(1) = ReAllSave.timeFh(4);
    flags(8) = true;
    send(pub{4}, humanData{4});
end

% Finish Serving 1
if any(timingData >= ReAllSave.timeF(37)) && ~flags(9)
    finserv = true;
    slowlier = true;
    humanData{1}.FinishServing(1) = ReAllSave.timeF(37);
    humanData{1}.ConfirmServing(1) = 1;
    humanData{1}.RobotVelocityProximity(1) = 0.5; % Slow
    humanData{1}.WaitingTime = repmat(0.5, 1, num_filling_boxes); % Waiting time high
    humanData{1}.RobotWaitingDistance(1) = 0.5;
    flags(9) = true;
    send(pub{1}, humanData{1});
end

if any(timingData >= ReAllSave.timeF(38)+60) && ~flags(10)
    finserv = true;
    humanData{2}.FinishServing(1) = ReAllSave.timeF(38)+60;
    humanData{2}.ConfirmServing(1) = 1;
    humanData{2}.RobotVelocityProximity(1) = -1; % Very Fast
    humanData{2}.WaitingTime = repmat(-0.5, 1, num_filling_boxes); % Waiting time low
    humanData{2}.RobotWaitingDistance(2) = 1;
    flags(10) = true;
    send(pub{2}, humanData{2});
    pause(0.1)
end

if any(timingData >= ReAllSave.timeF(39)) && ~flags(11)
    finserv = true;
    humanData{3}.FinishServing(1) = ReAllSave.timeF(39);
    humanData{3}.ConfirmServing(1) = 1;
    humanData{3}.RobotVelocityProximity(1) = 0.5; % Slow
    humanData{3}.WaitingTime = repmat(-0.5, 1, num_filling_boxes); % Waiting time low
    humanData{3}.RobotWaitingDistance(3) = 1.5;
    flags(11) = true;
    send(pub{3}, humanData{3});
end

if any(timingData >= ReAllSave.timeF(40)) && ~flags(12)
    finserv = true;
    humanData{4}.FinishServing(1) = ReAllSave.timeF(40);
    humanData{4}.ConfirmServing(1) = 1;
    %humanData{4}.RobotVelocityProximity(1) = 0.5; % Slow
    %humanData{4}.WaitingTime = repmat(1, 1, num_filling_boxes); % Waiting time high
    humanData{4}.WaitingTimeWeight = repmat(1, 1, num_filling_boxes);
    humanData{4}.RobotWaitingDistance(4) = 2;
    flags(12) = true;
    send(pub{4}, humanData{4});
end

% Start Filling 2
if any(timingData >= ReAllSave.timeSh(5)+15) && ~flags(13)
    humanData{1}.StartFilling(2) = ReAllSave.timeSh(5)+15;
    flags(13) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= ReAllSave.timeSh(6)+70) && ~flags(14)
    humanData{2}.StartFilling(2) = ReAllSave.timeSh(6)+70;
    flags(14) = true;
    send(pub{2}, humanData{2});
elseif any(timingData >= ReAllSave.timeSh(7)+20) && ~flags(15)
    humanData{3}.StartFilling(2) = ReAllSave.timeSh(7)+20;
    flags(15) = true;
    send(pub{3}, humanData{3});
elseif any(timingData >= ReAllSave.timeSh(8)) && ~flags(16)
    humanData{4}.StartFilling(2) = ReAllSave.timeSh(8);
    flags(16) = true;
    send(pub{4}, humanData{4});
end

% Finish Filling & Start Serving 2
if any(timingData >= ReAllSave.timeFh(5)-10) && ~flags(17)
    finfill = true;
    humanData{1}.FinishFilling(2) = ReAllSave.timeFh(5)-10;
    humanData{1}.ConfirmFilling(2) = 1;
    humanData{1}.StartServing(2) = ReAllSave.timeS(41);
    flags(17) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= ReAllSave.timeFh(6)+30) && ~flags(18)
    finfill = true;
    humanData{2}.FinishFilling(2) = ReAllSave.timeFh(6)+30;
    humanData{2}.ConfirmFilling(2) = 1;
    humanData{2}.StartServing(2) = ReAllSave.timeFh(6);
    flags(18) = true;
    send(pub{2}, humanData{2});
elseif any(timingData >= ReAllSave.timeFh(7)+30) && ~flags(19)
    finfill = true;
    humanData{3}.FinishFilling(2) = ReAllSave.timeFh(7)+30;
    humanData{3}.ConfirmFilling(2) = 1;
    humanData{3}.StartServing(2) = ReAllSave.timeS(43);
    flags(19) = true;
    send(pub{3}, humanData{3});
elseif any(timingData >= ReAllSave.timeFh(8)) && ~flags(20)
    finfill = true;
    humanData{4}.FinishFilling(2) = ReAllSave.timeFh(8);
    humanData{4}.ConfirmFilling(2) = 1;
    humanData{4}.StartServing(2) = ReAllSave.timeFh(8);
    flags(20) = true;
    send(pub{4}, humanData{4});
end

% Finish Serving 2
if any(timingData >= ReAllSave.timeF(41)) && ~flags(21)
    finserv = true;
    humanData{1}.FinishServing(2) = ReAllSave.timeF(41);
    humanData{1}.ConfirmServing(2) = 1;
    humanData{1}.RobotVelocityProximity(2) = 0.5; % Slow
    humanData{1}.WaitingTime = repmat(1, 1, num_filling_boxes); % Waiting time too high
    humanData{1}.RobotWaitingDistance(2) = 1;
    flags(21) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= ReAllSave.timeF(42)) && ~flags(22)
    finserv = true;
    humanData{2}.FinishServing(2) = ReAllSave.timeF(42);
    humanData{2}.ConfirmServing(2) = 1;
    humanData{2}.RobotVelocityProximity(2) = -0.5; % Fast
    humanData{2}.WaitingTime = repmat(-0.5, 1, num_filling_boxes); % Waiting time low
    humanData{2}.RobotWaitingDistance(2) = 1;
    flags(22) = true;
    send(pub{2}, humanData{2});
elseif any(timingData >= ReAllSave.timeF(43)) && ~flags(23)
    finserv = true;
    humanData{3}.FinishServing(2) = ReAllSave.timeF(43);
    humanData{3}.ConfirmServing(2) = 1;
    humanData{3}.RobotVelocityProximity(2) = 0; % Ok
    humanData{3}.WaitingTime = repmat(-0.5, 1, num_filling_boxes); % high
    humanData{3}.RobotWaitingDistance(3) = 1.5;
    flags(23) = true;
    send(pub{3}, humanData{3});
elseif any(timingData >= ReAllSave.timeF(44)) && ~flags(24)
    finserv = true;
    humanData{4}.FinishServing(2) = ReAllSave.timeF(44);
    humanData{4}.ConfirmServing(2) = 1;
    humanData{4}.RobotVelocityProximity(2) = 0; % Ok
    %humanData{4}.WaitingTime = repmat(-1, 1, num_filling_boxes); % Ok
    humanData{4}.WaitingTimeWeight = repmat(0.5, 1, num_filling_boxes);
    humanData{4}.RobotWaitingDistance(4) = 2;
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
    finfill = true;
    humanData{1}.FinishFilling(3) = ReAllSave.timeFh(9);
    humanData{1}.ConfirmFilling(3) = 1;
    humanData{1}.StartServing(3) = ReAllSave.timeFh(9);
    flags(29) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= ReAllSave.timeFh(10)) && ~flags(30)
    finfill = true;
    humanData{2}.FinishFilling(3) = ReAllSave.timeFh(10);
    humanData{2}.ConfirmFilling(3) = 1;
    humanData{2}.StartServing(3) = ReAllSave.timeFh(10);
    flags(30) = true;
    send(pub{2}, humanData{2});
elseif any(timingData >= ReAllSave.timeFh(11)+30) && ~flags(31)
    finfill = true;
    humanData{3}.FinishFilling(3) = ReAllSave.timeFh(11)+30;
    humanData{3}.ConfirmFilling(3) = 1;
    humanData{3}.StartServing(3) = ReAllSave.timeFh(11);
    flags(31) = true;
    send(pub{3}, humanData{3});
elseif any(timingData >= ReAllSave.timeFh(12)-10) && ~flags(32)
    finfill = true;
    humanData{4}.FinishFilling(3) = ReAllSave.timeFh(12);
    humanData{4}.ConfirmFilling(3) = 1;
    humanData{4}.StartServing(3) = ReAllSave.timeFh(12);
    flags(32) = true;
    send(pub{4}, humanData{4});
end

% Finish Serving 3
if any(timingData >= ReAllSave.timeF(45)) && ~flags(33)
    finserv = true;
    humanData{1}.FinishServing(3) = ReAllSave.timeF(45);
    humanData{1}.ConfirmServing(3) = 1;
    humanData{1}.RobotVelocityProximity(3) = 0; % Ok
    humanData{1}.WaitingTime = repmat(0, 1, num_filling_boxes); 
    humanData{1}.RobotWaitingDistance(3) = 0.5;
    flags(33) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= ReAllSave.timeF(46)) && ~flags(34)
    finserv = true;
    humanData{2}.FinishServing(3) = ReAllSave.timeF(46);
    humanData{2}.ConfirmServing(3) = 1;
    humanData{2}.RobotVelocityProximity(3) = 0; % Ok
    humanData{2}.WaitingTime = repmat(0, 1, num_filling_boxes); 
    humanData{2}.RobotWaitingDistance(3) = 1.5;
    flags(34) = true;
    send(pub{2}, humanData{2});
elseif any(timingData >= ReAllSave.timeF(47)) && ~flags(35)
    finserv = true;
    humanData{3}.FinishServing(3) = ReAllSave.timeF(47);
    humanData{3}.ConfirmServing(3) = 1;
    humanData{3}.RobotVelocityProximity(3) = 0; % Ok
    humanData{3}.WaitingTime = repmat(0, 1, num_filling_boxes);
    humanData{3}.RobotWaitingDistance(3) = 1.5;
    flags(35) = true;
    send(pub{3}, humanData{3});
elseif any(timingData >= ReAllSave.timeF(48)) && ~flags(36)
    finserv = true;
    humanData{4}.FinishServing(3) = ReAllSave.timeF(48);
    humanData{4}.ConfirmServing(3) = 1;
    humanData{4}.RobotVelocityProximity(3) = 0; % Ok
    %humanData{4}.WaitingTime = repmat(0, 1, num_filling_boxes);
    humanData{4}.RobotWaitingDistance(4) = 2;
    flags(36) = true;
    send(pub{4}, humanData{4});
end
