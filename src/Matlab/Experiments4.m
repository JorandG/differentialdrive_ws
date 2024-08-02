global timingData humanData pub M
global flags

% Initialize ProximityTaskFeedback, ProximityTaskWeights, ProximityTaskDurations if not already
if ~exist('ProximityTaskFeedback', 'var')
    ProximityTaskFeedback = [];
end
if ~exist('ProximityTaskWeights', 'var')
    ProximityTaskWeights = [];
end
if ~exist('ProximityTaskDurations', 'var')
    ProximityTaskDurations = [];
end

% Start Filling 1
if any(timingData >= 8) && ~flags(1)
    humanData{1}.StartFilling(1) = 8;
    flags(1) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= 10) && ~flags(2)
    humanData{2}.StartFilling(1) = 10;
    flags(2) = true;
    send(pub{2}, humanData{2});
elseif any(timingData >= 5) && ~flags(3)
    humanData{3}.StartFilling(1) = 5;
    flags(3) = true;
    send(pub{3}, humanData{3});
end

% Finish Filling & Start Serving 1
if any(timingData >= 215) && ~flags(4)
    humanData{1}.FinishFilling(1) = 215;
    humanData{1}.ConfirmFilling(1) = 1;
    humanData{1}.StartServing(1) = 215;
    flags(4) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= 280) && ~flags(5)
    humanData{2}.FinishFilling(1) = 280
    humanData{2}.ConfirmFilling(1) = 1;
    humanData{2}.StartServing(1) = 280;
    flags(5) = true;
    send(pub{2}, humanData{2});
elseif any(timingData >= 240) && ~flags(6)
    humanData{3}.FinishFilling(1) = 240
    humanData{3}.ConfirmFilling(1) = 1;
    humanData{3}.StartServing(1) = 240;
    flags(6) = true;
    send(pub{3}, humanData{3});
elseif any(timingData >= 250) && ~flags(21)
    humanData{4}.ConfirmFilling(1) = 1;    
    flags(21) = true;
    send(pub{4}, humanData{4});     
end

% Finish Serving 1
if any(timingData >= 222) && ~flags(7)
    humanData{1}.FinishServing(1) = 222;
    humanData{1}.ConfirmServing(1) = 1;    
    humanData{1}.RobotVelocityProximity(1) = 1; % Very Slow
    humanData{1}.WaitingTime(1) = 1; % Waiting time too high
    humanData{1}.RobotWaitingDistance(1) = 0.5;
    flags(7) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= 302) && ~flags(8)
    humanData{2}.FinishServing(1) = 302;
    humanData{2}.ConfirmServing(1) = 1;    
    humanData{2}.RobotVelocityProximity(1) = -1; % Very Fast
    humanData{2}.WaitingTime(1) = -1; % Waiting time too low
    humanData{2}.RobotWaitingDistance(2) = 1;
    flags(8) = true;
    send(pub{2}, humanData{2});     
elseif any(timingData >= 250) && ~flags(9)
    humanData{3}.FinishServing(1) = 250;
    humanData{3}.ConfirmServing(1) = 1;    
    humanData{3}.RobotVelocityProximity(1) = 0.5; % Slow
    humanData{3}.WaitingTime(1) = 1; % Waiting time high
    humanData{3}.RobotWaitingDistance(2) = 1;
    flags(9) = true;
    send(pub{3}, humanData{3});     
elseif any(timingData >= 435) && ~flags(20)
    humanData{4}.ConfirmServing(1) = 1;    
    flags(20) = true;
    send(pub{4}, humanData{4});     
end

% Start Filling 2
if any(timingData >= 440) && ~flags(10)
    disp('start filling 21')
    humanData{1}.StartFilling(2) = 440;
    flags(10) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= 310) && ~flags(11)
    disp('start filling 22')
    humanData{2}.StartFilling(2) = 310;
    flags(11) = true;
    send(pub{2}, humanData{2});
elseif any(timingData >= 352) && ~flags(22)
    disp('start filling 22')
    humanData{3}.StartFilling(2) = 352;
    flags(22) = true;
    send(pub{3}, humanData{3});
end

% Finish Filling & Start Serving 2
if any(timingData >= 580) && ~flags(12)
    humanData{1}.FinishFilling(2) = 580;
    humanData{1}.ConfirmFilling(2) = 1;
    humanData{1}.StartServing(2) = 580;
    flags(12) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= 595) && ~flags(13)
    humanData{2}.FinishFilling(2) = 595;
    humanData{2}.ConfirmFilling(2) = 1;
    humanData{2}.StartServing(2) = 595;
    flags(13) = true;
    send(pub{2}, humanData{2});
elseif any(timingData >= 550) && ~flags(14)
    humanData{3}.FinishFilling(2) = 550;
    humanData{3}.ConfirmFilling(2) = 1;
    humanData{3}.StartServing(2) = 550;
    flags(14) = true;
    send(pub{3}, humanData{3});
end

% Finish Serving 2
if any(timingData >= 587) && ~flags(15)
    humanData{1}.FinishServing(2) = 587;
    humanData{1}.ConfirmServing(2) = 1;
    humanData{1}.RobotVelocityProximity(2) = 0.5; % Slow
    humanData{1}.WaitingTime(2) = 0.5; % Waiting time high
    humanData{1}.RobotWaitingDistance(2) = 1;
    flags(15) = true;
    send(pub{1}, humanData{1});
elseif any(timingData >= 608) && ~flags(16)
    humanData{2}.FinishServing(2) = 608;
    humanData{2}.ConfirmServing(2) = 1;  
    humanData{2}.RobotVelocityProximity(2) = -0.5; % Fast
    humanData{2}.WaitingTime(2) = -0.5; % Waiting time low
    humanData{2}.RobotWaitingDistance(2) = 1;
    flags(16) = true;
    send(pub{2}, humanData{2});
elseif any(timingData >= 560) && ~flags(17)
    humanData{3}.FinishServing(2) = 560;
    humanData{3}.ConfirmServing(2) = 1;  
    humanData{3}.RobotVelocityProximity(2) = 0; % Ok
    humanData{3}.WaitingTime(2) = 0; % Ok
    humanData{3}.RobotWaitingDistance(2) = 1;
    flags(17) = true;
    send(pub{3}, humanData{3});
end
% 
% % Start Filling 3
% if any(timingData >= 410) && ~flags(13)
%     disp('start Filling 31')
%     humanData{1}.StartFilling(3) = 410;
%     flags(13) = true;
%     send(pub{1}, humanData{1});
% elseif any(timingData >= 435) && ~flags(14)
%     disp('start Filling 32')
%     humanData{2}.StartFilling(3) = 435;
%     flags(14) = true;
%     send(pub{2}, humanData{2});
% end
% 
% % Finish Filling & Start Serving 3
% if any(timingData >= 505) && ~flags(15)
%     humanData{1}.FinishFilling(3) = 505;
%     humanData{1}.ConfirmFilling(3) = 1;
%     humanData{1}.StartServing(3) = 535;
%     flags(15) = true;
%     send(pub{1}, humanData{1});
% elseif any(timingData >= 608) && ~flags(16)
%     humanData{2}.FinishFilling(3) = 608;
%     humanData{2}.ConfirmFilling(3) = 1;
%     humanData{2}.StartServing(3) = 608;
%     flags(16) = true;
%     send(pub{2}, humanData{2});
% end
% 
% % Finish Serving 3
% if any(timingData >= 522) && ~flags(17)
%     humanData{1}.FinishServing(3) = 522;
%     humanData{1}.ConfirmServing(3) = 1;
%     humanData{1}.RobotVelocityProximity(3) = 0; % Ok
%     humanData{1}.WaitingTime(3) = 0; % Ok
%     humanData{1}.RobotWaitingDistance(3) = 0.5;
%     flags(17) = true;
%     send(pub{1}, humanData{1});
% elseif any(timingData >= 618) && ~flags(18)
%     humanData{2}.FinishServing(3) = 618;
%     humanData{2}.ConfirmServing(3) = 1;  
%     humanData{2}.RobotVelocityProximity(3) = 0; % Ok
%     humanData{2}.WaitingTime(3) = 0; % Ok
%     humanData{2}.RobotWaitingDistance(3) = 1.5;
%     flags(18) = true;
%     send(pub{2}, humanData{2});
% end
% 
% % Final Feedback Collection
% if any(timingData >= 890) && ~flags(19)
%     flags(19) = true;
%     ProximityTaskFeedback = [ProximityTaskFeedback, humanData{1}.RobotVelocityProximity(3)'];
%     ProximityTaskFeedback = [ProximityTaskFeedback, humanData{2}.RobotVelocityProximity(3)'];
%     ProximityTaskWeights = [ProximityTaskWeights, humanData{1}.RobotVelocityProximityWeight(3)'];
%     ProximityTaskWeights = [ProximityTaskWeights, humanData{2}.RobotVelocityProximityWeight(3)'];
%     ProximityTaskDurations = [ProximityTaskDurations, (ReAllSave.timeF(idx_approaching_tasks) - ReAllSave.timeS(idx_approaching_tasks))];
% end
