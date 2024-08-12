function [stationManagement,outputValues,simValues] = updateKPILTE(activeIDsTXLTE,indexInActiveIDsOnlyLTE,awarenessID_LTE,neighborsID_LTE,timeManagement,stationManagement,positionManagement,sinrManagement,outputValues,outParams,simParams,appParams,phyParams,simValues)

% Error detection (up to RawMax)
% Each line corresponds to an error [TX, RX, BR, distance] within RawMax
errorMatrixRawMax = findErrors(activeIDsTXLTE,indexInActiveIDsOnlyLTE,neighborsID_LTE,sinrManagement,stationManagement,positionManagement,phyParams);

% Error detection (within each value of Raw)
for iPhyRaw=1:length(phyParams.Raw)
    
    errorMatrix = errorMatrixRawMax(errorMatrixRawMax(:,4)<phyParams.Raw(iPhyRaw),:);
%     errorMatrix = errorMatrixRawMax(errorMatrixRawMax(:,4)<100000,:);

    % Call function to create awarenessMatrix
    % [#Correctly transmitted beacons, #Errors, #Neighbors]
    awarenessMatrix = counterTX(activeIDsTXLTE,indexInActiveIDsOnlyLTE,awarenessID_LTE(:,:,iPhyRaw),errorMatrix);

    % Number of errors
    for iChannel = 1:phyParams.nChannels
        for pckType = 1:appParams.nPckTypes
            Nerrors = length(errorMatrix( (stationManagement.pckType(errorMatrix(:,1))==pckType & stationManagement.vehicleChannel(errorMatrix(:,1))==iChannel),1));
            outputValues.NerrorsLTE(iChannel,pckType,iPhyRaw) = outputValues.NerrorsLTE(iChannel,pckType,iPhyRaw) + Nerrors;
            outputValues.NerrorsTOT(iChannel,pckType,iPhyRaw) = outputValues.NerrorsTOT(iChannel,pckType,iPhyRaw) + Nerrors;
        end
    end
    
    % Number of transmitted beacons
    for iChannel = 1:phyParams.nChannels
        for pckType = 1:appParams.nPckTypes
            NtxBeacons = sum(awarenessMatrix((stationManagement.pckType(activeIDsTXLTE)==pckType & stationManagement.vehicleChannel(activeIDsTXLTE)==iChannel),3));
            outputValues.NtxBeaconsLTE(iChannel,pckType,iPhyRaw) = outputValues.NtxBeaconsLTE(iChannel,pckType,iPhyRaw) + NtxBeacons;
            outputValues.NtxBeaconsTOT(iChannel,pckType,iPhyRaw) = outputValues.NtxBeaconsTOT(iChannel,pckType,iPhyRaw) + NtxBeacons;
        end
    end
    
    % Number of correctly transmitted beacons
    for iChannel = 1:phyParams.nChannels
        for pckType = 1:appParams.nPckTypes
            NcorrectlyTxBeacons = sum(awarenessMatrix((stationManagement.pckType(activeIDsTXLTE)==pckType & stationManagement.vehicleChannel(activeIDsTXLTE)==iChannel),1));
            outputValues.NcorrectlyTxBeaconsLTE(iChannel,pckType,iPhyRaw) = outputValues.NcorrectlyTxBeaconsLTE(iChannel,pckType,iPhyRaw) + NcorrectlyTxBeacons;
            outputValues.NcorrectlyTxBeaconsTOT(iChannel,pckType,iPhyRaw) = outputValues.NcorrectlyTxBeaconsTOT(iChannel,pckType,iPhyRaw) + NcorrectlyTxBeacons;
        end
    end
    
    % Compute update delay (if enabled)
    if outParams.printUpdateDelay
        [simValues.updateTimeMatrixLTE,outputValues.updateDelayCounterLTE] = countUpdateDelay(stationManagement,iPhyRaw,activeIDsTXLTE,indexInActiveIDsOnlyLTE,stationManagement.BRid,appParams.NbeaconsF,awarenessID_LTE(:,:,iPhyRaw),errorMatrix,timeManagement.timeNow,simValues.updateTimeMatrixLTE,outputValues.updateDelayCounterLTE,outParams.delayResolution,outParams.enableUpdateDelayHD);
    end

    % Compute data age (if enabled)
    if outParams.printDataAge
        [simValues.dataAgeTimestampMatrixLTE,outputValues.dataAgeCounterLTE] = countDataAge(stationManagement,iPhyRaw,timeManagement,activeIDsTXLTE,indexInActiveIDsOnlyLTE,stationManagement.BRid,appParams.NbeaconsF,awarenessID_LTE(:,:,iPhyRaw),errorMatrix,timeManagement.timeNow,simValues.dataAgeTimestampMatrixLTE,outputValues.dataAgeCounterLTE,outParams.delayResolution,appParams);
    end

    % Compute packet delay (if enabled)
    if outParams.printPacketDelay
        outputValues.packetDelayCounterLTE = countPacketDelay(stationManagement,iPhyRaw,activeIDsTXLTE,timeManagement.timeNow,timeManagement.timeGeneratedPacketInTxLTE,awarenessMatrix(:,1),outputValues.packetDelayCounterLTE,outParams.delayResolution);
    end

    % Compute power control allocation (if enabled)
    if outParams.printPowerControl
        error('Output not updated in v5');
        %   % Convert linear PtxERP values to Ptx in dBm
        %	Ptx_dBm = 10*log10((phyParams.PtxERP_RB*appParams.RBsBeacon)/(2*phyParams.Gt))+30;
        %	outputValues.powerControlCounter = countPowerControl(IDvehicleTX,Ptx_dBm,outputValues.powerControlCounter,outParams.powerResolution);
    end

    % Update matrices needed for PRRmap creation in urban scenarios (if enabled)
    if simParams.typeOfScenario==2 && outParams.printPRRmap
        simValues = counterMap(iPhyRaw,simValues,stationManagement.activeIDsLTE,indexInActiveIDsOnlyLTE,activeIDsTXLTE,awarenessID_LTE(:,:,iPhyRaw),errorMatrix);
    end

end

% Count distance details for distances up to the maximum awareness range (if enabled)
if outParams.printPacketReceptionRatio
    outputValues.distanceDetailsCounterLTE = countDistanceDetails(indexInActiveIDsOnlyLTE,activeIDsTXLTE,neighborsID_LTE,stationManagement.neighborsDistanceLTE,errorMatrixRawMax,outputValues.distanceDetailsCounterLTE,stationManagement,outParams,appParams,phyParams);
end