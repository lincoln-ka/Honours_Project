logFile    = 'C:\Users\linky\OneDrive\Documents\Mission Planner\sitl\flightaxis\logs\00000389.BIN';
outputFile = 'C:\Users\linky\OneDrive\Documents\2026\IND_PROJ\Honours_Project\Logs\output389.mat';

binToMat(logFile, outputFile);

function binToMat(logFile, outputFile)
    reader = ardupilotreader(logFile);

    msgTable = reader.AvailableMessageTable;
    if ~isempty(msgTable.Properties.RowNames)
        msgNames = msgTable.Properties.RowNames;
    else
        msgNames = cellstr(msgTable.Name);
    end

    data = struct();
    for i = 1:numel(msgNames)
        msgType = msgNames{i};
        if strcmp(msgType, 'BAD_DATA')
            continue
        end

        tt = readMsg(reader, msgType);
        if isempty(tt)
            continue
        end

        fieldNames = tt.Properties.VariableNames;
        msgStruct = struct();
        for j = 1:numel(fieldNames)
            fname = fieldNames{j};
            msgStruct.(matlab.lang.makeValidName(fname)) = tt.(fname);
        end

        data.(matlab.lang.makeValidName(msgType)) = msgStruct;
    end

    save(outputFile, '-struct', 'data');
end
