clear
clc

logFile = fopen("RobotSerial.log");

logLines = [];
logLine = fgetl(logFile);

while ischar(logLine)
    if(~isnan(str2double(logLine(1))))
        xy = split(logLine, ", ");
        x = str2double(xy(1));
        y = str2double(xy(2));

        logLines = [logLines; x, y];
    end
    logLine = fgetl(logFile);
end

fclose(logFile);

plot(logLines(:,1), logLines(:,2))