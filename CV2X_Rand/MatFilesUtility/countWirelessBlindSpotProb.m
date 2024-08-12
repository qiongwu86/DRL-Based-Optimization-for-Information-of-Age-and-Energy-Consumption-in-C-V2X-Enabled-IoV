function wirelessBlindSpotCounter = countWirelessBlindSpotProb(updateTimeMatrix,wirelessBlindSpotCounter,elapsedTime)
% Function to store delay events for wireless blind spot computation

% Build timeDiff matrix
timeDiffMatrix = updateTimeMatrix;
timeDiffMatrix(timeDiffMatrix>0) = elapsedTime-timeDiffMatrix(timeDiffMatrix>0);

% For every time interval (multiples of Tbeacon)
for i = 1:length(wirelessBlindSpotCounter)
    % Count number of delay events larger or equal than time interval
    wirelessBlindSpotCounter(i,2) = wirelessBlindSpotCounter(i,2) + sum(timeDiffMatrix(:)>=wirelessBlindSpotCounter(i,1));
    % Count number of delay events shorter than time interval
    wirelessBlindSpotCounter(i,3) = wirelessBlindSpotCounter(i,3) + sum(timeDiffMatrix(:)>0 & timeDiffMatrix(:)<wirelessBlindSpotCounter(i,1));
end
