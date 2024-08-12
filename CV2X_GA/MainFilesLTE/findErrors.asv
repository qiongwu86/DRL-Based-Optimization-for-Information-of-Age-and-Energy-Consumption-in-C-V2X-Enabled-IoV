function errorMatrix = findErrors(IDvehicleTXLTE,indexVehicleTX,neighborsID,sinrManagement,stationManagement,positionManagement,phyParams)
% Detect wrongly decoded beacons and create Error Matrix
% [ID TX, ID RX, BRid, distance]

distance = positionManagement.distanceReal(stationManagement.vehicleState(stationManagement.activeIDs)==100,stationManagement.vehicleState(stationManagement.activeIDs)==100);

Ntx = length(IDvehicleTXLTE);              % Number of tx vehicles
errorMatrix = zeros(Ntx*Ntx-1,4);          % Initialize error matrix
Nerrors = 0;                               % Initialize number of errors

for i = 1:Ntx

    % Find indexes of receiving vehicles in neighborsID
    indexNeighborsRX = find(neighborsID(indexVehicleTX(i),:));
    
    for j = 1:length(indexNeighborsRX)
        % If received beacon SINR is lower than the threshold
        %if sinrManagement.neighborsSINR(i,indexNeighborsRX(j)) < phyParams.gammaMinLTE
        % randomSINRthreshold = sinrV(randi(length(sinrV)))
        sinrThreshold=(phyParams.LOS(i,indexNeighborsRX(j))*phyParams.sinrVectorLTE_LOS(randi(length(phyParams.sinrVectorLTE_LOS)))+... %if LOS
            (1-phyParams.LOS(i,indexNeighborsRX(j)))*phyParams.sinrVectorLTE_NLOS(randi(length(phyParams.sinrVectorLTE_NLOS))));  %if NLOS
% % % % % % %         phyParams.LOS是？
        if sinrManagement.neighborsSINRaverageLTE(i,indexNeighborsRX(j)) < sinrThreshold
            
            IDvehicleRX = neighborsID(indexVehicleTX(i),indexNeighborsRX(j));
            Nerrors = Nerrors + 1;
            errorMatrix(Nerrors,1) = IDvehicleTXLTE(i);
            errorMatrix(Nerrors,2) = IDvehicleRX;
            errorMatrix(Nerrors,3) = stationManagement.BRid(IDvehicleTXLTE(i));
            errorMatrix(Nerrors,4) = distance(indexVehicleTX(i),stationManagement.activeIDsLTE==IDvehicleRX);
%                 fid = fopen('temp.xls','a');
%                 fprintf(fid,'%d\t%d\t%.3f\t%f\t%f\t%f\n',IDvehicleRX,IDvehicleTX(i),distance(indexVehicleTX(i),IDvehicle==IDvehicleRX),...
%                     sinrManagement.neighPowerUsefulLastLTE(i,indexNeighborsRX(j)), sinrManagement.neighPowerInterfLastLTE(i,indexNeighborsRX(j)),neighborsSINRaverageLTE(i,indexNeighborsRX(j)));
%                 fclose(fid);
        end
    end
end

delIndex = errorMatrix(:,1)==0;
errorMatrix(delIndex,:) = [];

end
