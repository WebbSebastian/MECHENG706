clear
clc

logFile = fopen("RobotSerial.log");

logLines = [];
logLine = fgetl(logFile);

while ischar(logLine)
    if(~isnan(str2double(logLine(1))))
         logLines = [logLines; str2double(logLine(1:5)), str2double(logLine(7:12))];
    end
    logLine = fgetl(logFile);
end

fclose(logFile);

plot(logLines(:,1), logLines(:,2))