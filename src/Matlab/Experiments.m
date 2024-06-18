global timingData humanData pub

%Start Filling 1
if any(timingData >= 8) && any(timingData <= 10)
    humanData{1}.StartFilling(1) = timingData;
    send(pub{1}, humanData{1});
elseif any(timingData >= 10) && any(timingData <= 12)
    humanData{2}.StartFilling(1) = timingData;
    send(pub{2}, humanData{2});
end

%Finish Filling & Start Serving 1
if any(timingData >= 115) && any(timingData <= 117)
    humanData{1}.ConfirmFilling(1) = 1;
    humanData{1}.StartServing(1) = timingData;
    send(pub{1}, humanData{1});
elseif any(timingData >= 170) && any(timingData <= 172)
    humanData{2}.ConfirmFilling(1) = 1;
    humanData{2}.StartServing(1) = timingData;
    send(pub{2}, humanData{2});
end

%Finish Serving 1
if any(timingData >= 121) && any(timingData <= 123)
    humanData{1}.ConfirmServing(1) = 1;    
    humanData{1}.RobotVelocityProximity(1) = 0.8; %Very Fast
    send(pub{1}, humanData{1});
elseif any(timingData >= 177) && any(timingData <= 179)
    humanData{2}.ConfirmServing(1) = 1;    
    humanData{2}.RobotVelocityProximity(1) = 0.2; %Very Slow
    send(pub{2}, humanData{2});
end

%Start Filling 2
if any(timingData >= 180) && any(timingData <= 182)
    humanData{1}.StartFilling(2) = timingData;
    send(pub{1}, humanData{1});
elseif any(timingData >= 200) && any(timingData <= 202)
    humanData{2}.StartFilling(2) = timingData;
    send(pub{2}, humanData{2});
end

%Finish Filling & Start Serving 2
if any(timingData >= 300) && any(timingData <= 302)
    humanData{1}.ConfirmFilling(2) = 1;
    humanData{1}.StartServing(2) = timingData;
    send(pub{1}, humanData{1});
elseif any(timingData >= 365) && any(timingData <= 367)
    humanData{2}.ConfirmFilling(2) = 1;
    humanData{2}.StartServing(2) = timingData;
    send(pub{2}, humanData{2});
end

%Finish Serving 2
if any(timingData >= 307) && any(timingData <= 309)
    humanData{1}.ConfirmServing(2) = 1;
    humanData{1}.RobotVelocityProximity(2) = 0.5; %Ok
    send(pub{1}, humanData{1});
elseif any(timingData >= 375) && any(timingData <= 377)
    humanData{2}.ConfirmServing(2) = 1;  
    humanData{2}.RobotVelocityProximity(2) = 0.5; %Ok
    send(pub{2}, humanData{2});
end

